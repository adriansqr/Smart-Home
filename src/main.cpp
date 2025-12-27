/*
  SmartHome - ESP32 + Blynk (Option C) - Robust WiFi Handling
  - 4 relays (V0..V3)
  - 2 LEDs to mirror two lights
  - I2C LCD (16x2)
  - Buzzer
  - Uses BLYNK callbacks + periodic Blynk.syncVirtual to ensure cloud updates are applied
*/

#define BLYNK_TEMPLATE_ID "TMPL6DPwOBZJY"
#define BLYNK_TEMPLATE_NAME "SmartHome"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <LiquidCrystal_I2C.h>
#include <Ticker.h>

// ===== CONFIG =====

const char* WIFI_SSID = "Daniel@time";
const char* WIFI_PASS = "zurina@time";
const char* BLYNK_AUTH = "lwCLoMljKjkk8wIPttzbcpeGvTDOv-nJ";


// ===== PIN MAP =====
const uint8_t RELAY_PINS[4] = {14, 27, 26, 25}; // relays 1..4 (active LOW)
const uint8_t LED_PINS[2]   = {4, 5};          // LED1 mirrors relay1, LED2 mirrors relay2
const uint8_t BUZZER_PIN    = 18;              // buzzer output
const uint8_t LCD_ADDR      = 0x27;
const uint8_t LCD_COLS      = 16;
const uint8_t LCD_ROWS      = 2;

// ===== OBJECTS =====
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
BlynkTimer blynkTimer; // timer from Blynk library
bool relayState[4] = { false, false, false, false };

// ===== UTIL =====
void beep(uint16_t ms = 120) {
  #if defined(ESP32)
    ledcSetup(0, 2000, 8);        // channel 0, 2kHz, 8-bit
    ledcAttachPin(BUZZER_PIN, 0);
    ledcWriteTone(0, 1000);
    delay(ms);
    ledcWriteTone(0, 0);
  #else
    tone(BUZZER_PIN, 1000, ms);
  #endif
}

void showLCDStatus() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.printf("R1:%s R2:%s", relayState[0] ? "ON " : "OFF", relayState[1] ? "ON " : "OFF");
  lcd.setCursor(0,1);
  lcd.printf("R3:%s R4:%s", relayState[2] ? "ON " : "OFF", relayState[3] ? "ON " : "OFF");
}

// Update hardware, LCD, and Blynk cloud
void updateRelay(uint8_t idx, bool on) {
  if (idx >= 4) return;
  relayState[idx] = on;

  // Relays active LOW
  digitalWrite(RELAY_PINS[idx], on ? LOW : HIGH);

  // Update LEDs
  if (idx < 2) digitalWrite(LED_PINS[idx], on ? HIGH : LOW);

  // Push status to cloud
  Blynk.virtualWrite(V0 + idx, on ? 1 : 0);

  // Update LCD & serial
  showLCDStatus();
  Serial.printf("Relay %u -> %s\n", idx+1, on ? "ON" : "OFF");
}

// Periodic sync from cloud
void syncAllVirtualPins() {
  Blynk.syncVirtual(V0, V1, V2, V3);
  Serial.println("Requested Blynk.syncVirtual(V0..V3)");
}

// ===== BLYNK WRITE HANDLERS =====
BLYNK_WRITE(V0) { updateRelay(0, param.asInt() != 0); }
BLYNK_WRITE(V1) { updateRelay(1, param.asInt() != 0); }
BLYNK_WRITE(V2) { updateRelay(2, param.asInt() != 0); }
BLYNK_WRITE(V3) { updateRelay(3, param.asInt() != 0); }

// ===== WIFI + BLYNK HELPER =====
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  lcd.clear();
  lcd.print("Connecting WiFi");
  Serial.print("Connecting to WiFi");
  
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - startAttemptTime > 15000) { // retry every 15s
      Serial.println("\nRetrying WiFi...");
      WiFi.disconnect(true);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      startAttemptTime = millis();
      lcd.clear();
      lcd.print("Retrying WiFi");
      beep(100);
    }
  }
  
  Serial.println("\nWiFi Connected!");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("WiFi Connected");
  lcd.setCursor(0,1); lcd.print(WiFi.localIP().toString());
}

void connectBlynk() {
  lcd.clear(); lcd.print("Connecting Blynk");
  Blynk.config(BLYNK_AUTH);
  Blynk.connect();

  unsigned long blynkStart = millis();
  while (!Blynk.connected() && millis() - blynkStart < 10000) {
    Blynk.run();
    delay(100);
    Serial.print(".");
  }

  if (Blynk.connected()) {
    lcd.clear(); lcd.print("Blynk Connected");
    Serial.println("\nBlynk connected!");
  } else {
    lcd.clear(); lcd.print("Blynk Failed");
    Serial.println("\nFailed to connect Blynk");
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(200);

  // GPIO init
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], HIGH);
  }
  for (uint8_t i = 0; i < 2; ++i) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }
  pinMode(BUZZER_PIN, OUTPUT);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0); lcd.print("SmartHome Boot");
  delay(700);

  // Connect WiFi and Blynk
  connectWiFi();
  connectBlynk();

  // Start with all relays OFF
  for (uint8_t i = 0; i < 4; ++i) updateRelay(i, false);

  // Sync every 7s to catch IFTTT updates
  blynkTimer.setInterval(7000L, syncAllVirtualPins);

  // Initial sync
  delay(500);
  syncAllVirtualPins();
}

// ===== LOOP =====
void loop() {
  // Auto reconnect WiFi if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost! Reconnecting...");
    connectWiFi();
  }

  // Auto reconnect Blynk if disconnected
  if (!Blynk.connected()) {
    Serial.println("Blynk lost! Reconnecting...");
    connectBlynk();
  }

  Blynk.run();
  blynkTimer.run();
}
