/*
  SmartHome - ESP32 + Blynk
  - 2 Relays (V0, V1)
  - 1 Fan Servo (V2) -> SWEEPS back & forth on Pin 26
  - 1 Door Servo (V3) -> Open/Close on Pin 25
  - Auto-close Door logic
  - LCD 16x2
*/

#define BLYNK_TEMPLATE_ID "TMPL6DPwOBZJY"
#define BLYNK_TEMPLATE_NAME "SmartHome"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <LiquidCrystal_I2C.h>
#include <Ticker.h>
#include <Servo.h>

// ===== CONFIG ===== 
const char* WIFI_SSID  = "Daniel@time";
const char* WIFI_PASS  = "zurina@time";
const char* BLYNK_AUTH = "lwCLoMljKjkk8wIPttzbcpeGvTDOv-nJ";

// ===== PIN MAP =====
const uint8_t RELAY_PINS[2]   = {27, 14};   // Relays
const uint8_t LED_PINS[2]     = {4, 5};     // LEDs
const uint8_t FAN_SERVO_PIN   = 26;         // Fan Servo (Sweeping)
const uint8_t DOOR_SERVO_PIN  = 25;         // Door Servo
const uint8_t LCD_ADDR        = 0x27;

// ===== OBJECTS =====
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
BlynkTimer blynkTimer;
Servo doorServo;
Servo fanServo;

// ===== STATE =====
bool relayState[2] = {false, false};
bool doorOpen = false;
bool fanOn = false;

// Timers
int autoCloseTimerID = -1;
int fanSweepTimerID  = -1; // New timer for the sweeping motion

// Servo Positions
int currentDoorPos = 0;
int currentFanPos  = 0;
int fanDirection   = 5;    // Speed/Step size for fan (larger = faster)

// ===== LCD =====
void showLCDStatus() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.printf("R1:%s R2:%s",
    relayState[0] ? "ON " : "OFF",
    relayState[1] ? "ON " : "OFF");

  lcd.setCursor(0,1);
  lcd.printf("Fan:%s D:%s",
    fanOn ? "ACT" : "OFF",
    doorOpen ? "OPN" : "CLS");
}

// ===== RELAY CONTROL =====
void updateRelay(uint8_t idx, bool on) {
  if (idx >= 2) return;
  relayState[idx] = on;
  digitalWrite(RELAY_PINS[idx], on ? LOW : HIGH);
  digitalWrite(LED_PINS[idx], on ? HIGH : LOW);
  Blynk.virtualWrite(V0 + idx, on);
  showLCDStatus();
}

// ===== DOOR MOVEMENT (Standard Smooth) =====
void moveDoorSmooth(int target) {
  if (currentDoorPos == target) return;
  int step = (currentDoorPos < target) ? 1 : -1;
  for (int pos = currentDoorPos; pos != target; pos += step) {
    doorServo.write(pos);
    delay(6); 
  }
  currentDoorPos = target;
  doorServo.write(target);
}

// ===== FAN SWEEP LOGIC (Called repeatedly) =====
void sweepFan() {
  // If accidentally called when off, stop
  if (!fanOn) return; 

  // Move the position
  currentFanPos += fanDirection;

  // Check bounds and reverse direction
  if (currentFanPos >= 180) {
    currentFanPos = 180;
    fanDirection = -5; // Reverse
  } else if (currentFanPos <= 0) {
    currentFanPos = 0;
    fanDirection = 5;  // Forward
  }

  fanServo.write(currentFanPos);
}

// ===== FAN CONTROL (Start/Stop Sweep) =====
void updateFan(bool on) {
  if (fanOn == on) return;
  fanOn = on;

  if (on) {
    // START SWEEPING
    // Run sweepFan() every 30ms. No delay() allowed here!
    if (fanSweepTimerID != -1) blynkTimer.deleteTimer(fanSweepTimerID);
    fanSweepTimerID = blynkTimer.setInterval(30L, sweepFan);
    Serial.println("Fan: STARTED Sweeping");
  } else {
    // STOP SWEEPING
    if (fanSweepTimerID != -1) {
      blynkTimer.deleteTimer(fanSweepTimerID);
      fanSweepTimerID = -1;
    }
    // Return to 0
    fanServo.write(0);
    currentFanPos = 0;
    Serial.println("Fan: STOPPED");
  }
  
  Blynk.virtualWrite(V2, on);
  showLCDStatus();
}

// ===== DOOR CONTROL =====
void autoCloseDoor() {
  if (!doorOpen) return;
  doorOpen = false;
  moveDoorSmooth(0);
  Blynk.virtualWrite(V3, 0);
  showLCDStatus();
  Serial.println("Door AUTO-CLOSED");
}

void updateDoor(bool open) {
  if (doorOpen == open) return;
  doorOpen = open;
  moveDoorSmooth(open ? 90 : 0);
  Blynk.virtualWrite(V3, open);
  showLCDStatus();

  if (open) {
    if (autoCloseTimerID != -1) blynkTimer.deleteTimer(autoCloseTimerID);
    autoCloseTimerID = blynkTimer.setTimeout(10000L, autoCloseDoor);
  } else {
    if (autoCloseTimerID != -1) {
      blynkTimer.deleteTimer(autoCloseTimerID);
      autoCloseTimerID = -1;
    }
  }
}

// ===== BLYNK HANDLERS =====
BLYNK_WRITE(V0) { updateRelay(0, param.asInt()); }
BLYNK_WRITE(V1) { updateRelay(1, param.asInt()); }
BLYNK_WRITE(V2) { updateFan(param.asInt()); }
BLYNK_WRITE(V3) { updateDoor(param.asInt()); }

// ===== WIFI & SETUP =====
void syncAllVirtualPins() {
  Blynk.syncVirtual(V0, V1, V2, V3);
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  lcd.clear(); lcd.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) delay(500);
  lcd.clear(); lcd.print("WiFi OK");
}

void connectBlynk() {
  lcd.clear(); lcd.print("Connecting Blynk");
  Blynk.config(BLYNK_AUTH);
  Blynk.connect();
  while (!Blynk.connected()) { Blynk.run(); delay(100); }
  lcd.clear(); lcd.print("Blynk OK");
}

void setup() {
  Serial.begin(115200);

  // Relays
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], HIGH);
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }

  // Servos
  doorServo.attach(DOOR_SERVO_PIN);
  doorServo.write(0);
  
  fanServo.attach(FAN_SERVO_PIN);
  fanServo.write(0);
  currentFanPos = 0;

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.print("SmartHome V3");
  delay(800);

  connectWiFi();
  connectBlynk();

  // Reset logic
  updateRelay(0, false);
  updateRelay(1, false);
  updateFan(false);
  updateDoor(false);

  blynkTimer.setInterval(7000L, syncAllVirtualPins);
  syncAllVirtualPins();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!Blynk.connected()) connectBlynk();

  Blynk.run();
  blynkTimer.run(); // Important: This handles the fan sweep timer
}