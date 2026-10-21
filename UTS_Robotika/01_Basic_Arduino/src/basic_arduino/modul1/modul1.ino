/*
 * Pemrograman Kit iMCLab Berbasis Blynk IoT + OTA (Over-The-Air)
 * UTS Robotika - Wireless Firmware Update & Teleoperation
 */

#define BLYNK_TEMPLATE_ID   "TMPL6D171v5HG"      // Ganti dengan Template ID Blynk kamu
#define BLYNK_TEMPLATE_NAME "iMCLab Robotika"  // Ganti dengan Template Name Blynk kamu
#define BLYNK_AUTH_TOKEN    "tulh3OTHhwdetIE_OWRnbncafYogXjoA"

// Versi Firmware untuk Identifikasi OTA
#define BLYNK_FIRMWARE_VERSION "1.0.1"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ArduinoOTA.h>  // Library bawaan ESP32 untuk Wireless OTA

// Konfigurasi Wi-Fi
char ssid[] = "KOS MBAH ROHMAN";      // Ganti dengan SSID Hotspot/WiFi
char pass[] = "Satuduatiga";  // Ganti dengan Password WiFi

// Pin Hardware Kit iMCLab
constexpr uint8_t LED_PIN = 2;
constexpr uint8_t MOTOR_IN1_PIN = 27;
constexpr uint8_t MOTOR_IN2_PIN = 26;
constexpr uint8_t MOTOR_PWM_PIN = 12;

constexpr uint8_t ENCODER_A_PIN = 34;
constexpr uint8_t ENCODER_B_PIN = 35;

constexpr uint32_t PWM_FREQUENCY = 30000;
constexpr uint8_t PWM_RESOLUTION = 8;
constexpr uint8_t PWM_CHANNEL = 0;

// Variabel Kontrol & Telemetry
volatile long encoderTicks = 0;
long lastTicks = 0;
bool pwmReady = false;
bool motorEnable = true;

float targetRPM = 0.0;
float currentRPM = 0.0;
float currentAngle = 0.0;
float integralError = 0.0;
float lastError = 0.0;

// Parameter PID Speed
float Kp = 1.2;
float Ki = 0.5;
float Kd = 0.05;

constexpr float CPR = 330.0;
BlynkTimer timer;

// Interrupt Service Routine (ISR) Encoder
void IRAM_ATTR readEncoder() {
  if (digitalRead(ENCODER_B_PIN) == HIGH) {
    encoderTicks++;
  } else {
    encoderTicks--;
  }
}

// Fungsi Kendali Motor
void setMotorSpeed(int speed) {
  if (!motorEnable) {
    speed = 0;
  }
  
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

// BLYNK WRITE: Slider Target RPM (V0)
BLYNK_WRITE(V0) {
  targetRPM = param.asFloat();
  Serial.print("[Blynk] Target RPM: ");
  Serial.println(targetRPM);
}

// BLYNK WRITE: Emergency Stop Switch (V2)
BLYNK_WRITE(V2) {
  motorEnable = (param.asInt() == 1);
  if (!motorEnable) {
    setMotorSpeed(0);
    Serial.println("[Blynk] Emergency Stop ACTIVE!");
  }
}

// Kirim Data Telemetry ke Blynk (setiap 100ms)
void sendTelemetryToBlynk() {
  Blynk.virtualWrite(V1, currentRPM);
  Blynk.virtualWrite(V3, currentAngle);
}

// Setup Fitur OTA
void setupOTA() {
  ArduinoOTA.setHostname("iMCLab-ESP32-OTA");
  ArduinoOTA.setPassword("imclab123"); // Password keamanan upload OTA

  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "Sketch" : "FileSystem";
    Serial.println("[OTA] Proses flashing dimulai: " + type);
    setMotorSpeed(0); // Matikan motor demi keamanan saat flashing
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA] Flash Selesai! Restarting ESP32...");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  Serial.println("[OTA] Service OTA Berhasil Diaktifkan.");
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

  // Inisialisasi Koneksi Blynk & OTA
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  setupOTA();

  timer.setInterval(100L, sendTelemetryToBlynk);
  Serial.println("System Ready. Firmware Version: " BLYNK_FIRMWARE_VERSION);
}

unsigned long lastPidTime = 0;

void loop() {
  Blynk.run();
  ArduinoOTA.handle(); // Memeriksa & memproses permintaan update firmware OTA
  timer.run();

  // Loop PID Speed Control (setiap 50 ms)
  unsigned long currentTime = millis();
  if (currentTime - lastPidTime >= 50) {
    float dt = (currentTime - lastPidTime) / 1000.0;
    lastPidTime = currentTime;

    long currentTicks = encoderTicks;
    long deltaTicks = currentTicks - lastTicks;
    lastTicks = currentTicks;

    currentRPM = (deltaTicks / CPR) * (60.0 / dt);
    currentAngle = (currentTicks / CPR) * 360.0;

    float error = targetRPM - currentRPM;
    integralError += error * dt;
    integralError = constrain(integralError, -255.0, 255.0);
    float derivativeError = (error - lastError) / dt;
    lastError = error;

    float output = (Kp * error) + (Ki * integralError) + (Kd * derivativeError);
    int pwrOutput = constrain((int)output, -255, 255);

    setMotorSpeed(pwrOutput);
  }
}