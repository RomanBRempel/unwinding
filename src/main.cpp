#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <soc/gpio_struct.h>

// --- ПИНЫ ESP32 ---
static const uint8_t POT_PIN = 34;      // GPIO34 (ADC1)
static const uint8_t ENABLE_PIN = 4;    // D4
static const uint8_t DIR_PIN = 2;       // D2
static const uint8_t STEP_PIN = 15;     // D15

// --- НАСТРОЙКИ ДВИЖЕНИЯ ---
static const uint8_t MICROSTEPS = 4;
static const float STEPS_PER_REV = 200.0f * MICROSTEPS;
// Максимальная скорость в RPM (обороты в минуту)
static const float MAX_RPM = 600.0f;
static const float MAX_SPEED_SPS = (MAX_RPM * STEPS_PER_REV) / 60.0f;
static const float BASE_RPM = 400.0f;
static const float BASE_SPEED_SPS = (BASE_RPM * STEPS_PER_REV) / 60.0f;
// Ускорение для плавного разгона/торможения (шаги/с²)
// Увеличено до 16000 для поддержки высоких скоростей с микрошагами (MICROSTEPS=8)
// При MICROSTEPS=8 нужно 8x больше ускорения для той же скорости разгона в RPM
static const float ACCELERATION_SPS2 = 16000.0f;

// Сглаживание потенциометра
static const float POT_SMOOTHING = 0.2f;
static const uint32_t CONTROL_INTERVAL_MS = 10;
static const uint32_t SPEED_SAMPLE_MS = 100;
static const uint32_t SERIAL_LOG_MS = 200;

// --- Wi-Fi and Web UI ---
static const char* WIFI_SSID = "RBR_WiFi";
static const char* WIFI_PASSWORD = "87770759";

// --- Step pulse timer ---
static const uint32_t STEP_TIMER_HZ = 20000;
static const uint32_t STEP_TIMER_US = 1000000UL / STEP_TIMER_HZ;


// --- PID для удержания значения потенциометра ---
// ESP32 использует 12-битный АЦП (0..4095)
static const float POT_SETPOINT = 2048.0f; 

static const float PID_KP = 10.0f;
static const float PID_KI = 0.5f;
static const float PID_KD = 0.1f;
static const float PID_OUTPUT_LIMIT = MAX_SPEED_SPS - BASE_SPEED_SPS;
static const int POT_HOLD_BAND = 20; // Зона удержания без движения (x4 для 12-бит)
static const int POT_STOP_THRESHOLD = 20; // Если потенциометр около 0 - стоп (x4 для 12-бит)

AsyncWebServer server(80);

portMUX_TYPE stepperMux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t* stepTimer = nullptr;
volatile float isrSpeedSps = 0.0f;
volatile float isrAccumulator = 0.0f;
volatile uint32_t isrStepCount = 0;

// --- Глобальные переменные ---
float smoothedPot = 0.0f;
float currentSpeed = 0.0f;
float targetSpeed = 0.0f;
float pidIntegral = 0.0f;
float pidLastError = 0.0f;
uint32_t lastPidTime = 0;
uint32_t lastControlUpdate = 0;
uint32_t lastSerialLog = 0;
uint32_t lastSpeedSample = 0;
uint32_t lastSpeedUpdate = 0;
float actualSpeedSps = 0.0f;
int rawPot = 0;
float pidP = 0.0f;
float pidI = 0.0f;
float pidD = 0.0f;
float pidOutput = 0.0f;

static float spsToRpm(float speedSps) {
  return (speedSps * 60.0f) / STEPS_PER_REV;
}

static void startWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("WiFi connecting to ");
  Serial.println(WIFI_SSID);
  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    if (millis() - start > 15000) {
      break;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.print("WiFi connect failed, status: ");
    Serial.println(static_cast<int>(WiFi.status()));
  }
}

static void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>RBR System Status</title>
  <style>
    :root { color-scheme: light; }
    body { font-family: "Segoe UI", Arial, sans-serif; background: #f6f7fb; margin: 0; }
    header { background: #1e2a38; color: #fff; padding: 16px 20px; }
    main { padding: 16px 20px; display: grid; gap: 12px; grid-template-columns: repeat(auto-fit, minmax(240px, 1fr)); }
    .card { background: #fff; border-radius: 8px; padding: 12px 14px; box-shadow: 0 2px 6px rgba(0,0,0,0.06); }
    .card h3 { margin: 0 0 8px; font-size: 14px; color: #5c6b7a; }
    .value { font-size: 20px; font-weight: 600; color: #1e2a38; }
    .sub { font-size: 12px; color: #6b7b8c; }
    footer { padding: 10px 20px; font-size: 12px; color: #6b7b8c; }
  </style>
</head>
<body>
  <header>
    <h2>RBR System Status</h2>
  </header>
  <main>
    <div class="card"><h3>Potentiometer</h3><div class="value" id="pot">-</div><div class="sub" id="potSub">-</div></div>
    <div class="card"><h3>Target Speed</h3><div class="value" id="target">-</div><div class="sub" id="targetSub">-</div></div>
    <div class="card"><h3>Actual Speed</h3><div class="value" id="actual">-</div><div class="sub" id="actualSub">-</div></div>
    <div class="card"><h3>PID Output</h3><div class="value" id="pidOut">-</div><div class="sub" id="pidTerms">-</div></div>
    <div class="card"><h3>Error</h3><div class="value" id="error">-</div><div class="sub" id="timing">-</div></div>
  </main>
  <footer>Auto-refreshing every 500 ms</footer>
  <script>
    async function refresh() {
      try {
        const res = await fetch('/status');
        if (!res.ok) return;
        const d = await res.json();
        document.getElementById('pot').textContent = `${d.pot_smoothed.toFixed(1)} / ${d.pot_raw}`;
        document.getElementById('potSub').textContent = `Setpoint: ${d.pot_setpoint}`;
        document.getElementById('target').textContent = `${d.target_rpm.toFixed(1)} rpm`;
        document.getElementById('targetSub').textContent = `${d.target_sps.toFixed(1)} steps/s`;
        document.getElementById('actual').textContent = `${d.actual_rpm.toFixed(1)} rpm`;
        document.getElementById('actualSub').textContent = `${d.actual_sps.toFixed(1)} steps/s`;
        document.getElementById('pidOut').textContent = d.pid_output.toFixed(2);
        document.getElementById('pidTerms').textContent = `P: ${d.pid_p.toFixed(2)} I: ${d.pid_i.toFixed(2)} D: ${d.pid_d.toFixed(2)}`;
        document.getElementById('error').textContent = d.error.toFixed(1);
        document.getElementById('timing').textContent = `Uptime: ${d.uptime_ms} ms`;
      } catch (e) {}
    }
    setInterval(refresh, 500);
    refresh();
  </script>
</body>
</html>
    )HTML";

    request->send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    const float targetRpm = spsToRpm(targetSpeed);
    const float actualRpm = spsToRpm(actualSpeedSps);
    String json;
    json.reserve(256);
    json += "{\"pot_raw\":" + String(rawPot);
    json += ",\"pot_smoothed\":" + String(smoothedPot, 1);
    json += ",\"pot_setpoint\":" + String(POT_SETPOINT, 0);
    json += ",\"error\":" + String(pidLastError, 1);
    json += ",\"target_sps\":" + String(targetSpeed, 2);
    json += ",\"target_rpm\":" + String(targetRpm, 2);
    json += ",\"actual_sps\":" + String(actualSpeedSps, 2);
    json += ",\"actual_rpm\":" + String(actualRpm, 2);
    json += ",\"pid_p\":" + String(pidP, 2);
    json += ",\"pid_i\":" + String(pidI, 2);
    json += ",\"pid_d\":" + String(pidD, 2);
    json += ",\"pid_output\":" + String(pidOutput, 2);
    json += ",\"uptime_ms\":" + String(millis());
    json += "}";
    request->send(200, "application/json", json);
  });

  server.begin();
  Serial.println("Web server started");
}

void IRAM_ATTR onStepTimer() {
  portENTER_CRITICAL_ISR(&stepperMux);
  const float speed = isrSpeedSps;
  if (speed > 0.0f) {
    isrAccumulator += speed / static_cast<float>(STEP_TIMER_HZ);
    while (isrAccumulator >= 1.0f) {
      isrAccumulator -= 1.0f;
      GPIO.out_w1ts = (1UL << STEP_PIN);
      GPIO.out_w1tc = (1UL << STEP_PIN);
      isrStepCount++;
    }
  } else {
    isrAccumulator = 0.0f;
  }
  portEXIT_CRITICAL_ISR(&stepperMux);
}


void setup() {
  Serial.begin(115200);

  // Настройка АЦП для ESP32
  analogReadResolution(12);  // 12-битное разрешение (0-4095)
  analogSetAttenuation(ADC_11db);  // Полный диапазон 0-3.3V для всех пинов

  // Настройка конкретного пина ADC
  adcAttachPin(POT_PIN);

  // Инициализация
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);

  // Инициализация сглаженного значения
  rawPot = analogRead(POT_PIN);
  smoothedPot = rawPot;

  startWiFi();
  setupWebServer();

  stepTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(stepTimer, &onStepTimer, true);
  timerAlarmWrite(stepTimer, STEP_TIMER_US, true);
  timerAlarmEnable(stepTimer);

  delay(1000);
}

void loop() {
  const uint32_t now = millis();
  
  if (now - lastControlUpdate >= CONTROL_INTERVAL_MS) {
    lastControlUpdate = now;
    // --- ЧТЕНИЕ И СГЛАЖИВАНИЕ ПОТЕНЦИОМЕТРА ---
    rawPot = analogRead(POT_PIN);
    smoothedPot = (POT_SMOOTHING * rawPot) + ((1.0f - POT_SMOOTHING) * smoothedPot);

    // --- PID УДЕРЖАНИЕ ПОТЕНЦИОМЕТРА ---
    float error = smoothedPot - POT_SETPOINT;

    if (smoothedPot <= POT_STOP_THRESHOLD) {
      targetSpeed = 0.0f;
      pidIntegral = 0.0f;
      pidLastError = 0.0f;
      pidP = 0.0f;
      pidI = 0.0f;
      pidD = 0.0f;
      pidOutput = 0.0f;
      lastPidTime = now;
    } else if (abs((int)error) <= POT_HOLD_BAND) {
      targetSpeed = BASE_SPEED_SPS;
      pidIntegral = 0.0f;
      pidLastError = 0.0f;
      pidP = 0.0f;
      pidI = 0.0f;
      pidD = 0.0f;
      pidOutput = 0.0f;
      lastPidTime = now;
    } else {
      if (lastPidTime == 0) {
        lastPidTime = now;
      }

      float dt = (now - lastPidTime) / 1000.0f;
      if (dt > 0.0f) {
        pidIntegral += error * dt;

        if (PID_KI > 0.0f) {
          float integralLimit = PID_OUTPUT_LIMIT / PID_KI;
          if (pidIntegral > integralLimit) pidIntegral = integralLimit;
          if (pidIntegral < -integralLimit) pidIntegral = -integralLimit;
        }

        float derivative = (error - pidLastError) / dt;
        pidP = PID_KP * error;
        pidI = PID_KI * pidIntegral;
        pidD = PID_KD * derivative;
        float output = pidP + pidI + pidD;

        // Ограничение выхода PID для симметричного ускорения и замедления
        if (output > PID_OUTPUT_LIMIT) output = PID_OUTPUT_LIMIT;
        if (output < -PID_OUTPUT_LIMIT) output = -PID_OUTPUT_LIMIT;

        pidOutput = output;

        targetSpeed = BASE_SPEED_SPS + output;
        if (targetSpeed < 0.0f) targetSpeed = 0.0f;
        if (targetSpeed > MAX_SPEED_SPS) targetSpeed = MAX_SPEED_SPS;
        pidLastError = error;
        lastPidTime = now;
      }
    }
  }

  if (lastSpeedUpdate == 0) {
    lastSpeedUpdate = now;
  }

  const float dtSpeed = (now - lastSpeedUpdate) / 1000.0f;
  if (dtSpeed > 0.0f) {
    const float maxDelta = ACCELERATION_SPS2 * dtSpeed;
    if (targetSpeed > currentSpeed) {
      currentSpeed = min(currentSpeed + maxDelta, targetSpeed);
    } else if (targetSpeed < currentSpeed) {
      currentSpeed = max(currentSpeed - maxDelta, targetSpeed);
    }
    lastSpeedUpdate = now;
  }

  float speedForIsr = currentSpeed;
  if (speedForIsr < 0.0f) {
    digitalWrite(DIR_PIN, HIGH);
    speedForIsr = -speedForIsr;
  } else {
    digitalWrite(DIR_PIN, LOW);
  }

  portENTER_CRITICAL(&stepperMux);
  isrSpeedSps = speedForIsr;
  portEXIT_CRITICAL(&stepperMux);

  if (now - lastSpeedSample >= SPEED_SAMPLE_MS) {
    uint32_t steps;
    portENTER_CRITICAL(&stepperMux);
    steps = isrStepCount;
    isrStepCount = 0;
    portEXIT_CRITICAL(&stepperMux);
    float dt = (now - lastSpeedSample) / 1000.0f;
    if (dt > 0.0f) {
      actualSpeedSps = steps / dt;
    }
    lastSpeedSample = now;
  }

  
}