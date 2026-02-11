#include <Arduino.h>
#include <AccelStepper.h>
#include <LiquidCrystal.h>

// --- ПИНЫ ---
static const uint8_t POT_PIN = A15;
static const uint8_t ENABLE_PIN = 28;
static const uint8_t DIR_PIN = 30;
static const uint8_t STEP_PIN = 32;

// LCD
static const uint8_t LCD_RS = 8, LCD_EN = 9, LCD_D4 = 4, LCD_D5 = 5, LCD_D6 = 6, LCD_D7 = 7;
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// --- НАСТРОЙКИ ДВИЖЕНИЯ ---
static const uint8_t MICROSTEPS = 8;
static const float STEPS_PER_REV = 200.0f * MICROSTEPS;
// Максимальная скорость в RPM (обороты в минуту)
// Для скоростей выше 300 RPM рекомендуется использовать микрошаги (MICROSTEPS > 1)
static const float MAX_RPM = 600.0f;
static const float MAX_SPEED_SPS = (MAX_RPM * STEPS_PER_REV) / 60.0f;
static const float BASE_RPM = 180.0f;
static const float BASE_SPEED_SPS = (BASE_RPM * STEPS_PER_REV) / 60.0f;
// Ускорение для плавного разгона/торможения (шаги/с²)
// Увеличено до 2000 для поддержки высоких скоростей (300+ RPM)
// Для ещё больших скоростей можно увеличить до 3000-5000
static const float ACCELERATION_SPS2 = 2000.0f;
// Целевая позиция для непрерывного движения (достаточно далеко для непрерывной работы)
static const long CONTINUOUS_MOVEMENT_STEPS = 1000000L;

// Сглаживание потенциометра
static const float POT_SMOOTHING = 0.2f;
static const uint32_t CONTROL_INTERVAL_MS = 10;
static const uint32_t SPEED_SAMPLE_MS = 100;
static const uint32_t LCD_INTERVAL_MS = 250;

// --- PID для удержания значения потенциометра ---
static const float POT_SETPOINT = 512.0f; // Целевое значение потенциометра (0..1023)
// Увеличен PID_KP для более агрессивной реакции (было 8.33)
// Значение 10.0 для баланса между быстротой и стабильностью
static const float PID_KP = 10.0f;
static const float PID_KI = 0.5f;
static const float PID_KD = 0.1f;
static const float PID_OUTPUT_LIMIT = MAX_SPEED_SPS - BASE_SPEED_SPS;
static const int POT_HOLD_BAND = 5; // Зона удержания без движения
static const int POT_STOP_THRESHOLD = 5; // Если потенциометр около 0 - стоп

// --- AccelStepper объект ---
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// --- Глобальные переменные ---
float smoothedPot = 0.0f;
uint32_t lastLcdUpdate = 0;
float currentSpeed = 0.0f;
float targetSpeed = 0.0f;
float pidIntegral = 0.0f;
float pidLastError = 0.0f;
uint32_t lastPidTime = 0;
uint32_t lastControlUpdate = 0;
long lastStepPos = 0;
uint32_t lastSpeedSample = 0;
float actualSpeedSps = 0.0f;

void setup() {
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Direct Control");
  lcd.setCursor(0, 1);
  lcd.print("Init...");

  // Инициализация
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  
  stepper.setMaxSpeed(MAX_SPEED_SPS);
  stepper.setAcceleration(ACCELERATION_SPS2);
  
  // Инициализация сглаженного значения
  smoothedPot = analogRead(POT_PIN);
  
  delay(1000);
  lcd.clear();
}

void loop() {
  const uint32_t now = millis();
  
  if (now - lastControlUpdate >= CONTROL_INTERVAL_MS) {
    lastControlUpdate = now;
    // --- ЧТЕНИЕ И СГЛАЖИВАНИЕ ПОТЕНЦИОМЕТРА ---
    int rawInput = analogRead(POT_PIN);
    smoothedPot = (POT_SMOOTHING * rawInput) + ((1.0f - POT_SMOOTHING) * smoothedPot);

    // --- PID УДЕРЖАНИЕ ПОТЕНЦИОМЕТРА ---
    float error = smoothedPot - POT_SETPOINT;

    if (smoothedPot <= POT_STOP_THRESHOLD) {
      targetSpeed = 0.0f;
      pidIntegral = 0.0f;
      pidLastError = 0.0f;
      lastPidTime = now;
    } else if (abs((int)error) <= POT_HOLD_BAND) {
      targetSpeed = BASE_SPEED_SPS;
      pidIntegral = 0.0f;
      pidLastError = 0.0f;
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
        float output = (PID_KP * error) + (PID_KI * pidIntegral) + (PID_KD * derivative);

        // Ограничение выхода PID для симметричного ускорения и замедления
        if (output > PID_OUTPUT_LIMIT) output = PID_OUTPUT_LIMIT;
        if (output < -PID_OUTPUT_LIMIT) output = -PID_OUTPUT_LIMIT;

        targetSpeed = BASE_SPEED_SPS + output;
        if (targetSpeed < 0.0f) targetSpeed = 0.0f;
        if (targetSpeed > MAX_SPEED_SPS) targetSpeed = MAX_SPEED_SPS;
        pidLastError = error;
        lastPidTime = now;
      }
    }
  }

  currentSpeed = targetSpeed;
  stepper.setMaxSpeed(targetSpeed);
  
  // Используем run() вместо runSpeed() для плавного ускорения
  // Устанавливаем далёкую целевую позицию, чтобы двигатель двигался непрерывно
  // с управляемым ускорением. Это предотвращает вибрацию при высоких частотах.
  if (targetSpeed > 0.0f) {
    if (stepper.distanceToGo() == 0) {
      stepper.moveTo(stepper.currentPosition() + CONTINUOUS_MOVEMENT_STEPS);
    }
    stepper.run();
  } else {
    // Если целевая скорость 0, останавливаемся
    stepper.stop();
    }

  if (now - lastSpeedSample >= SPEED_SAMPLE_MS) {
    long currentPos = stepper.currentPosition();
    long deltaSteps = currentPos - lastStepPos;
    float dt = (now - lastSpeedSample) / 1000.0f;
    if (dt > 0.0f) {
      actualSpeedSps = deltaSteps / dt;
    }
    lastStepPos = currentPos;
    lastSpeedSample = now;
  }
  
  // --- ОБНОВЛЕНИЕ LCD ---
 if (now - lastLcdUpdate >= LCD_INTERVAL_MS) {
    lastLcdUpdate = now;
    
    int rpmActual = abs((int)((actualSpeedSps * 60.0f) / STEPS_PER_REV));
    int rpmTarget = abs((int)((currentSpeed * 60.0f) / STEPS_PER_REV));
    char line0[17];
    char line1[17];
    
    lcd.setCursor(0, 0);
    snprintf(line0, sizeof(line0), "T:%4d A:%4d   ", rpmTarget, rpmActual);
    lcd.print(line0);

    lcd.setCursor(0, 1);
    snprintf(line1, sizeof(line1), "P:%4d SP:%4d  ", (int)smoothedPot, (int)POT_SETPOINT);
    lcd.print(line1);
  } 
}