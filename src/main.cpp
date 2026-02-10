#include <Arduino.h>
#include <AccelStepper.h>
#include <LiquidCrystal.h>

// --- Hardware pins ---
static const uint8_t POT_PIN = A15; // potentiometer must be on analog pin (A0-A15 on Mega)
static const uint8_t ENABLE_PIN = 28;
static const uint8_t DIR_PIN = 30;
static const uint8_t STEP_PIN = 32;

// LCD Keypad Shield (standard wiring)
static const uint8_t LCD_RS = 8;
static const uint8_t LCD_EN = 9;
static const uint8_t LCD_D4 = 4;
static const uint8_t LCD_D5 = 5;
static const uint8_t LCD_D6 = 6;
static const uint8_t LCD_D7 = 7;

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// --- Control parameters ---
static const float STEPS_PER_REV = 200.0f;      // NEMA23 full-step
static const float TARGET_RPM_AT_MID = 177.0f;  // desired speed at pot=512
static const float TARGET_STEPS_PER_SEC = (TARGET_RPM_AT_MID * STEPS_PER_REV) / 60.0f;
static const float MAX_SPEED = TARGET_STEPS_PER_SEC * 2.0f; // linear map 0..1023
static const uint32_t LCD_PERIOD_MS = 500;
static const float ACCELERATION = 2000.0f; // steps/sec^2 for smooth ramp
static const long RUN_DISTANCE = 1000000L; // large target for continuous motion
static const float POT_FILTER_ALPHA = 0.1f; // low-pass filter for pot noise
static const int POT_DEADBAND = 4;          // ignore tiny jitters around zero
static const float SPEED_SLEW_PER_SEC = 800.0f; // limit speed change rate

// --- State ---
int lastInput = 0;
float filteredInput = 0.0f;
float currentSpeed = 0.0f;
uint32_t lastControlMs = 0;
uint32_t lastLcdMs = 0;

int readPotValue() {
  // Analog read. If using analog pin, set POT_PIN to A0..A15.
  return analogRead(POT_PIN);
}

void applySpeed(float speed) {
  if (speed <= 0.0f) {
    stepper.stop();
    return;
  }
  stepper.enableOutputs();
  stepper.setMaxSpeed(speed);
  if (stepper.distanceToGo() == 0) {
    stepper.moveTo(stepper.currentPosition() + RUN_DISTANCE);
  }
}

void setup() {
  Serial.begin(115200);

  stepper.setEnablePin(ENABLE_PIN);
  stepper.setPinsInverted(false, false, true); // enable is active-low on most drivers
  stepper.setMinPulseWidth(3); // TB6600 needs a few microseconds pulse width
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(ACCELERATION);
  stepper.enableOutputs();

  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Unwinding");
  lcd.setCursor(0, 1);
  lcd.print("Init...");

  lastInput = readPotValue();
  lastControlMs = millis();
  delay(500);
}

void loop() {
  const uint32_t now = millis();

  // Control loop at ~50 Hz
  if (now - lastControlMs >= 20) {
    const uint32_t dtMs = now - lastControlMs;
    lastControlMs = now;
    const float dt = dtMs / 1000.0f;

    const int input = readPotValue();
    lastInput = input;

    filteredInput += (static_cast<float>(input) - filteredInput) * POT_FILTER_ALPHA;

    float desiredSpeed = 0.0f;
    if (filteredInput > POT_DEADBAND) {
      const float ratio = filteredInput / 1023.0f;
      desiredSpeed = ratio * MAX_SPEED;
      if (desiredSpeed > MAX_SPEED) desiredSpeed = MAX_SPEED;
    }

    const float maxDelta = SPEED_SLEW_PER_SEC * dt;
    if (desiredSpeed > currentSpeed + maxDelta) {
      currentSpeed += maxDelta;
    } else if (desiredSpeed < currentSpeed - maxDelta) {
      currentSpeed -= maxDelta;
    } else {
      currentSpeed = desiredSpeed;
    }

    applySpeed(currentSpeed);
  }

  // Run stepper with acceleration
  stepper.run();

  // LCD update (keep it slow to avoid stalling steps)
  if (now - lastLcdMs >= LCD_PERIOD_MS) {
    lastLcdMs = now;
    const int input = lastInput;

    char line0[17];
    const int speed = static_cast<int>(stepper.speed());
    snprintf(line0, sizeof(line0), "P:%4d S:%4d", input, speed);
    lcd.setCursor(0, 0);
    lcd.print(line0);

    lcd.setCursor(0, 1);
    lcd.print("RUN             ");
  }
}
