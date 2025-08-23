#include <Arduino.h>
#include <U8g2lib.h>

// OLED setup (I2C, adjust pins if needed)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

// ADC pin
#define BATTERY_PIN 34

// Voltage divider resistors
const float R1 = 100000.0;  // 100k
const float R2 = 47000.0;   // 47k

// Battery limits (2S Li-ion)
const float MAX_VOLTAGE = 8.4;   // Full
const float MIN_VOLTAGE = 6.0;   // Empty

// Read smoothed battery voltage
float readBatteryVoltage() {
  long sum = 0;
  const int samples = 20;   // averaging for stability
  for (int i = 0; i < samples; i++) {
    sum += analogRead(BATTERY_PIN);
    delay(5);
  }
  int rawADC = sum / samples;

  float adcVoltage = (rawADC / 4095.0) * 3.3; // pin voltage
  float batteryVoltage = adcVoltage * ((R1 + R2) / R2);

  return batteryVoltage;
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  u8g2.begin();
}

void loop() {
  float batteryVoltage = readBatteryVoltage();

  // Calculate %
  int percentage = round(((batteryVoltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100);
  if (percentage > 100) percentage = 100;
  if (percentage < 0) percentage = 0;

  // --- Serial Debug ---
  Serial.print("Battery Voltage: ");
  Serial.print(batteryVoltage, 2);
  Serial.print(" V | Battery %: ");
  Serial.print(percentage);
  Serial.println(" %");

  // --- OLED Display ---
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  // Draw battery outline (x, y, width, height)
  int x = 10, y = 20, w = 100, h = 20;
  u8g2.drawFrame(x, y, w, h);  // battery body
  u8g2.drawBox(x + w, y + 5, 4, h - 10); // battery tip

  // Fill based on percentage
  int fillWidth = (w - 2) * percentage / 100; // -2 for border
  u8g2.drawBox(x + 1, y + 1, fillWidth, h - 2);

  // Show text
  u8g2.setCursor(10, 60);
  u8g2.print("Charge: ");
  u8g2.print(percentage);
  u8g2.print(" %");

  u8g2.sendBuffer();

  delay(2000);
}
