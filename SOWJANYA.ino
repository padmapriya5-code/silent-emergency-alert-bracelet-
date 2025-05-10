#include <FS.h>
#include <SD_MMC.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "esp_camera.h"

// Camera configuration for ESP32-CAM (AI Thinker)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Sensor objects
MAX30105 particleSensor;
TinyGPSPlus gps;

// Hardware Serial for GSM
#define GSM_RX 01
#define GSM_TX 03
SoftwareSerial gsmSerial(GSM_RX, GSM_TX);

// GPS Serial
#define GPS_RX 17
#define GPS_TX 16
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

// Microphone pin
#define MIC_PIN 13

// Emergency button pin
#define BUTTON_PIN 13

// Variables
long lastBeat = 0;
float beatsPerMinute;
int beatAvg = 0;
bool emergencyTriggered = false;
String emergencyContacts[] = {"+919019118624", "8050624710", };
String lastLocation = "";
unsigned long lastGPSTransmission = 0;

void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    while (true);
  }
}

void setup() {
  Serial.begin(115200);
  
  // Initialize emergency button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Initialize SD card
  if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed");
  }

  // Initialize camera
  setupCamera();

  // Initialize MAX30102 sensor
  if (!particleSensor.begin()) {
    Serial.println("MAX30102 was not found");
  } else {
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0);
  }

  // Initialize GSM module
  gsmSerial.begin(9600);
  delay(1000);
  gsmSerial.println("AT");
  delay(1000);
  gsmSerial.println("AT+CMGF=1"); // Set SMS text mode
  delay(1000);

  // Initialize GPS
  gpsSerial.begin(9600);
}

void loop() {
  // Check emergency button
  if (digitalRead(BUTTON_PIN) == LOW || emergencyTriggered) {
    if (!emergencyTriggered) {
      emergencyTriggered = true;
      triggerEmergencyProtocol();
    }
  }

  // Health monitoring
  monitorHealth();

  // GPS tracking (every 5 seconds)
  if (millis() - lastGPSTransmission >= 5000) {
    updateGPSLocation();
    lastGPSTransmission = millis();
    
    // If in emergency mode, send location updates
    if (emergencyTriggered) {
      sendLocationUpdates();
    }
  }

  delay(100);
}

void triggerEmergencyProtocol() {
  Serial.println("Emergency triggered!");
  
  // Capture photo evidence
  captureAndSavePhoto();
  
  // Record audio evidence (5 seconds)
  recordAudio(5000);
  
  // Send emergency alerts
  sendEmergencyAlerts();
  
  // Start continuous location updates
  lastGPSTransmission = 0; // Force immediate update
}

void captureAndSavePhoto() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  File file = SD_MMC.open("/emergency.jpg", FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.write(fb->buf, fb->len);
  file.close();
  esp_camera_fb_return(fb);
  Serial.println("Photo saved to SD card");
}

void recordAudio(unsigned int duration) {
  unsigned long startTime = millis();
  File audioFile = SD_MMC.open("/emergency.wav", FILE_WRITE);
  
  // Simple WAV header (44 bytes)
  byte wavHeader[44] = {
    'R', 'I', 'F', 'F', 0x00, 0x00, 0x00, 0x00, 'W', 'A', 'V', 'E',
    'f', 'm', 't', ' ', 0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00,
    0x44, 0xAC, 0x00, 0x00, 0x88, 0x58, 0x01, 0x00, 0x02, 0x00, 0x10, 0x00,
    'd', 'a', 't', 'a', 0x00, 0x00, 0x00, 0x00
  };
  
  audioFile.write(wavHeader, 44);
  
  while (millis() - startTime < duration) {
    int micValue = analogRead(MIC_PIN);
    int16_t sample = map(micValue, 0, 4095, -32768, 32767);
    audioFile.write((byte*)&sample, 2);
  }
  
  // Update WAV header with actual sizes
  uint32_t fileSize = audioFile.size();
  uint32_t dataSize = fileSize - 44;
  
  audioFile.seek(4);
  audioFile.write((byte*)&fileSize, 4);
  
  audioFile.seek(40);
  audioFile.write((byte*)&dataSize, 4);
  
  audioFile.close();
  Serial.println("Audio recording saved");
}

void sendEmergencyAlerts() {
  String message = "EMERGENCY ALERT!\n";
  message += "Location: " + lastLocation + "\n";
  message += "Vital signs:\n";
  message += "Heart Rate: " + String(beatAvg) + " BPM\n";
  
  // Send to all emergency contacts
  for (int i = 0; i < sizeof(emergencyContacts)/sizeof(emergencyContacts[0]); i++) {
    sendSMS(emergencyContacts[i], message);
  }
  Serial.println("Emergency alerts sent");
}

void sendSMS(String number, String message) {
  gsmSerial.println("AT+CMGS=\"" + number + "\"");
  delay(1000);
  gsmSerial.print(message);
  delay(1000);
  gsmSerial.write(26); // CTRL+Z to send
  delay(1000);
}

void monitorHealth() {
  long irValue = particleSensor.getIR();
  
  if (irValue > 50000) {
    if (checkForBeat(irValue)) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60000 / delta;
      beatAvg = (beatAvg * 3 + beatsPerMinute) / 4;
      Serial.print("Heart Rate: ");
      Serial.print(beatAvg);
      Serial.println(" BPM");
    }
  }
}

void updateGPSLocation() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        lastLocation = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
        Serial.print("Location updated: ");
        Serial.println(lastLocation);
      } else {
        lastLocation = "Unknown";
      }
    }
  }
}

void sendLocationUpdates() {
  String message = "Current Location: " + lastLocation + "\n";
  message += "Health Status: " + String(beatAvg) + " BPM";
  
  for (int i = 0; i < sizeof(emergencyContacts)/sizeof(emergencyContacts[0]); i++) {
    sendSMS(emergencyContacts[i], message);
  }
  Serial.println("Location update sent");
}

bool checkForBeat(int sensorValue) {
  static long lastTime = 0;
  static int lastValue = 0;
  static int threshold = 2000;
  
  bool beatDetected = false;
  
  if (sensorValue > lastValue + threshold && millis() - lastTime > 200) {
    beatDetected = true;
    lastTime = millis();
  }
  
  lastValue = sensorValue;
  return beatDetected;
}