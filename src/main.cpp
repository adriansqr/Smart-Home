//Arduino Libraries
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Wifi Libraries + Credentials
#include <WiFi.h>
#include <WiFiClientSecure.h>
#define WIFI_SSID "Daniel@time"
#define WIFI_PASSWORD "zurina@time"

// Hardware Libraries
#include <ESP32Servo.h>
int lcdColumns = 16;
int lcdRows = 2;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

void setup() {
  Wire.begin();
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting wifi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
    Serial.println("\nConnected with IP: " + WiFi.localIP().toString());

  // LCD setup
  lcd.init();
  lcd.backlight();
  lcd.print("Speak to give commands!");
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}