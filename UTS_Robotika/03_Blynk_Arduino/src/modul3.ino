#include <Arduino.h>

constexpr uint8_t LED_PIN = 2;
constexpr uint8_t MOTOR_IN1_PIN = 27;
constexpr uint8_t MOTOR_IN2_PIN = 26;
constexpr uint8_t MOTOR_PWM_PIN = 12;

constexpr uint8_t ENCODER_A_PIN = 34;
constexpr uint8_t ENCODER_B_PIN = 35;

constexpr uint32_t PWM_FREQUENCY = 30000;
constexpr uint8_t PWM_RESOLUTION = 8;
constexpr uint8_t PWM_CHANNEL = 0;

volatile long encoderTicks = 0;
long lastTicks = 0;
bool pwmReady = false;

float targetRPM = 0.0;
float currentRPM = 0.0;
float integralError = 0.0;
float lastError = 0.0;

float Kp = 1.2;
float Ki = 0.5;
float Kd = 0.05;

constexpr float CPR = 330.0;

void IRAM_ATTR readEncoder() {
  if (digitalRead(ENCODER_B_PIN) == HIGH) {
    encoderTicks++;
  } else {
    encoderTicks--;
  }
}

void setMotorSpeed(int speed) {
  bool forward = speed >= 0;
  uint8_t duty = constrain(abs(speed), 0, 255);

  digitalWrite(MOTOR_IN1_PIN, forward ? HIGH : LOW);
  digitalWrite(MOTOR_IN2_PIN, forward ? LOW : HIGH);

  if (pwmReady) {
    #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
      ledcWrite(MOTOR_PWM_PIN, duty);
    #else
      ledcWrite(PWM_CHANNEL, duty);
    #endif
  }
  digitalWrite(LED_PIN, duty > 0 ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), readEncoder, RISING);

  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    pwmReady = ledcAttach(MOTOR_PWM_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  #else
    ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PWM_PIN, PWM_CHANNEL);
    pwmReady = true;
  #endif

  setMotorSpeed(0);
  Serial.println("MODUL 3 READY: Kirim target RPM (misal: 100, -150, 0) via Serial Monitor.");
}

unsigned long lastPidTime = 0;

void loop() {
  if (Serial.available() > 0) {
    targetRPM = Serial.parseFloat();
    if (Serial.read() == '\n') {
      Serial.print("Target RPM Set: ");
      Serial.println(targetRPM);
    }
  }

  unsigned long currentTime = millis();
  if (currentTime - lastPidTime >= 50) {
    float dt = (currentTime - lastPidTime) / 1000.0;
    lastPidTime = currentTime;

    long currentTicks = encoderTicks;
    long deltaTicks = currentTicks - lastTicks;
    lastTicks = currentTicks;

    currentRPM = (deltaTicks / CPR) * (60.0 / dt);

    float error = targetRPM - currentRPM;
    integralError += error * dt;
    integralError = constrain(integralError, -255.0, 255.0);
    float derivativeError = (error - lastError) / dt;
    lastError = error;

    float output = (Kp * error) + (Ki * integralError) + (Kd * derivativeError);
    int pwrOutput = constrain((int)output, -255, 255);

    setMotorSpeed(pwrOutput);

    Serial.print("TargetRPM:"); Serial.print(targetRPM);
    Serial.print(" CurrentRPM:"); Serial.print(currentRPM);
    Serial.print(" Error:"); Serial.print(error);
    Serial.print(" PWM:"); Serial.println(pwrOutput);
  }
}