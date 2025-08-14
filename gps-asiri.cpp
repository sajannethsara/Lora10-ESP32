#include <Arduino.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HardwareSerial.h>

// === OLED Configuration ===
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// === GPS Configuration ===
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
HardwareSerial GPS_Serial(2); // Use UART2
TinyGPSPlus gps;

// === Shared Variables ===
float latitude = 0.0;
float longitude = 0.0;
bool gpsFix = false;
SemaphoreHandle_t gpsMutex;

// === Task: Read GPS Data ===
void _GpsUpdateTask(void *parameter) {
  while (1) {
    while (GPS_Serial.available() > 0) {
      char c = GPS_Serial.read();
      gps.encode(c);

      if (gps.location.isUpdated()) {
        xSemaphoreTake(gpsMutex, portMAX_DELAY);
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        gpsFix = gps.location.isValid();
        xSemaphoreGive(gpsMutex);
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// === Task: Display on Serial and OLED ===
void displayTask(void *parameter) {
  char latBuf[20], lonBuf[20];
  int counter = 0;

  while (1) {
    xSemaphoreTake(gpsMutex, portMAX_DELAY);
    float lat = latitude;
    float lon = longitude;
    bool fix = gpsFix;
    xSemaphoreGive(gpsMutex);

    snprintf(latBuf, sizeof(latBuf), "Lat: %.6f", lat);
    snprintf(lonBuf, sizeof(lonBuf), "Lon: %.6f", lon);

    Serial.println("=========================");
    Serial.println(fix ? latBuf : "Lat: ---");
    Serial.println(fix ? lonBuf : "Lon: ---");
    Serial.print("Count: ");
    Serial.println(counter);
    Serial.println("=========================");

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 10);
    display.println(fix ? latBuf : "Lat: ---");
    display.setCursor(0, 30);
    display.println(fix ? lonBuf : "Lon: ---");
    display.setCursor(0, 50);
    display.print("Count: ");
    display.println(counter);
    display.display();

    counter++;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  Wire.begin(21, 22); // SDA, SCL for OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED init failed");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 20);
  display.println("Initializing...");
  display.display();

  gpsMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(gpsTask, "GPS Task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(displayTask, "Display Task", 4096, NULL, 1, NULL, 0);
}

// === Loop (not used with RTOS) ===
void loop() {
  // Empty — logic handled in FreeRTOS tasks
}