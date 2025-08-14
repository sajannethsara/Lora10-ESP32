// src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ---------- Display ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------- Pins / I2C ----------
#define SDA_PIN 21
#define SCL_PIN 22

// ---------- Sensors addresses ----------
#define QMC5883L_ADDR 0x0D  // magnetometer QMC5883L typical address
#define MPU6500_ADDR  0x68  // MPU6500/MPU6050

// ---------- GPS ----------
HardwareSerial SerialGPS(1); // UART1
TinyGPSPlus gps;

// ---------- Waypoint storage ----------
struct Coordinate { float lat; float lon; };
static const int MAX_WAYPOINTS = 100;
static const float SAVE_DIST_METERS = 5.0f;
static const float ARRIVE_DIST_METERS = 5.0f;

Coordinate path[MAX_WAYPOINTS];
int pathIndex = 0;
int targetIndex = -1;

// ---------- Shared variables ----------
volatile float heading = 0.0f;        // updated by CompassTask (deg)
volatile float navDistance = 0.0f;    // distance to current target (m)
volatile float navBearing  = 0.0f;    // bearing to current target (deg)
volatile float navHeading  = 0.0f;    // current compass heading (deg)

// ---------- Mutexes ----------
SemaphoreHandle_t i2cMutex = NULL;      // for Wire + display + magnetometer + accel
SemaphoreHandle_t gpsDataMutex = NULL;  // for path[] and pathIndex

// ---------- Task handles ----------
TaskHandle_t CompassTaskHandle = NULL;
TaskHandle_t GpsTaskHandle = NULL;
TaskHandle_t ReverseNavTaskHandle = NULL;

// ---------- Forward declarations ----------
void CompassTask(void* pvParameters);
void GpsTask(void* pvParameters);
void ReverseNavTask(void* pvParameters);
void initSensors();
void readMagnetometer(int16_t &mx_out, int16_t &my_out, int16_t &mz_out);
void readAccelerometer(int16_t &ax_out, int16_t &ay_out, int16_t &az_out);
float distanceBetween(float lat1, float lon1, float lat2, float lon2);
float calculateBearing(float lat1, float lon1, float lat2, float lon2);

void setup() {
  Serial.begin(115200);
  delay(20);
  Serial.println("System starting...");

  // create mutexes
  i2cMutex = xSemaphoreCreateMutex();
  gpsDataMutex = xSemaphoreCreateMutex();
  if (!i2cMutex || !gpsDataMutex) {
    Serial.println("Failed to create mutexes");
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // start I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // init display (under i2c mutex)
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      Serial.println("OLED not found at 0x3C");
    } else {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      display.println("Starting...");
      display.display();
    }
    xSemaphoreGive(i2cMutex);
  } else {
    Serial.println("Couldn't take i2cMutex to init display");
  }

  // Initialize GPS UART1 (RX pin 16, TX pin 17)
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("GPS serial started");

  // sensor init (QMC5883L + MPU wake) - do under i2cMutex
  initSensors();

  // Create tasks
  xTaskCreatePinnedToCore(CompassTask, "CompassTask", 4096, NULL, 1, &CompassTaskHandle, 1);
  xTaskCreatePinnedToCore(GpsTask,     "GpsTask",     4096, NULL, 1, &GpsTaskHandle,     1);
  xTaskCreatePinnedToCore(ReverseNavTask, "ReverseNav", 2048, NULL, 2, &ReverseNavTaskHandle, 0);

  Serial.println("Tasks created");
}

void loop() {
  // empty - all logic in tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// -------------------- CompassTask --------------------
void CompassTask(void* pvParameters) {
  const TickType_t delayTicks = pdMS_TO_TICKS(200); // update ~5Hz
  int16_t magX=0, magY=0, magZ=0;
  int16_t accX=0, accY=0, accZ=0;

  // small startup pause
  vTaskDelay(pdMS_TO_TICKS(500));

  for (;;) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      // read sensors
      readMagnetometer(magX, magY, magZ);
      readAccelerometer(accX, accY, accZ);

      // tilt compensation
      float axn = (float)accX;
      float ayn = (float)accY;
      float azn = (float)accZ;
      float norm = sqrt(axn*axn + ayn*ayn + azn*azn);
      if (norm == 0.0f) norm = 1.0f;
      axn /= norm; ayn /= norm; azn /= norm;

      float xh = (float)magX * azn - (float)magZ * axn;
      float yh = (float)magY * azn - (float)magZ * ayn;
      float newHeading = atan2(yh, xh) * 180.0f / PI;
      if (newHeading < 0) newHeading += 360.0f;

      heading = newHeading; // update global heading
      navHeading = newHeading; // also mirror to navHeading for reverse nav usage

      // DRAW to display
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print("HDG: "); display.print(heading, 1); display.println(" deg");
      display.print("MX: "); display.print(magX);
      display.print(" MY: "); display.print(magY);
      display.print(" MZ: "); display.println(magZ);

      // draw arrow to navBearing if reverse-nav active
      display.drawCircle(SCREEN_WIDTH/2, SCREEN_HEIGHT/2 + 5, 2, SSD1306_WHITE);

      int centerX = SCREEN_WIDTH / 2;
      int centerY = SCREEN_HEIGHT / 2 + 5;
      int arrowLen = 20;

      // If reverse navigation is active (denoted by targetIndex >=0),
      // draw arrow pointing to navBearing (bearing to target).
      float displayAngleDeg;
      if (xSemaphoreTake(gpsDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (targetIndex >= 0 && pathIndex > 0) {
          // use navBearing (set by ReverseNav task)
          displayAngleDeg = navBearing;
        } else {
          // show compass heading (north direction)
          displayAngleDeg = heading;
        }
        xSemaphoreGive(gpsDataMutex);
      } else {
        displayAngleDeg = heading;
      }

      float angleRad = displayAngleDeg * PI / 180.0f;
      int arrowX = centerX + (int)(arrowLen * sin(angleRad));
      int arrowY = centerY - (int)(arrowLen * cos(angleRad));
      display.drawLine(centerX, centerY, arrowX, arrowY, SSD1306_WHITE);

      // show nav distance if targetIndex valid
      if (xSemaphoreTake(gpsDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (targetIndex >= 0) {
          display.setCursor(0, 48);
          display.setTextSize(1);
          display.print("Back: ");
          display.print(navDistance, 1);
          display.print(" m");
          display.setCursor(90, 48);
          display.setTextSize(2);
          // print cardinal roughly:
          if (navHeading >= 337.5f || navHeading < 22.5f)       display.println("N");
          else if (navHeading >= 22.5f && navHeading < 67.5f)   display.println("NE");
          else if (navHeading >= 67.5f && navHeading < 112.5f)  display.println("E");
          else if (navHeading >= 112.5f && navHeading < 157.5f) display.println("SE");
          else if (navHeading >= 157.5f && navHeading < 202.5f) display.println("S");
          else if (navHeading >= 202.5f && navHeading < 247.5f) display.println("SW");
          else if (navHeading >= 247.5f && navHeading < 292.5f) display.println("W");
          else if (navHeading >= 292.5f && navHeading < 337.5f) display.println("NW");
        }
        xSemaphoreGive(gpsDataMutex);
      }

      display.display();
      xSemaphoreGive(i2cMutex);
    } else {
      Serial.println("CompassTask: couldn't take i2cMutex");
    }
    vTaskDelay(delayTicks);
  }
}

// -------------------- GPS task --------------------
void GpsTask(void* pvParameters) {
  float lastLat=0, lastLon=0;
  bool firstFix = false;

  for (;;) {
    // feed TinyGPS parser from UART1
    while (SerialGPS.available()) {
      gps.encode(SerialGPS.read());
    }

    if (gps.location.isUpdated()) {
      float currLat = gps.location.lat();
      float currLon = gps.location.lng();

      if (!firstFix) {
        if (xSemaphoreTake(gpsDataMutex, portMAX_DELAY) == pdTRUE) {
          if (pathIndex < MAX_WAYPOINTS) {
            path[pathIndex].lat = currLat;
            path[pathIndex].lon = currLon;
            pathIndex++;
            Serial.printf("GPS: first fix saved (%.6f, %.6f) idx=%d\n", currLat, currLon, pathIndex);
          }
          xSemaphoreGive(gpsDataMutex);
        }
        lastLat = currLat; lastLon = currLon;
        firstFix = true;
      } else {
        float d = distanceBetween(currLat, currLon, lastLat, lastLon);
        if (d >= SAVE_DIST_METERS && pathIndex < MAX_WAYPOINTS) {
          if (xSemaphoreTake(gpsDataMutex, portMAX_DELAY) == pdTRUE) {
            path[pathIndex].lat = currLat;
            path[pathIndex].lon = currLon;
            pathIndex++;
            Serial.printf("GPS: saved (%.6f,%.6f) idx=%d d=%.2f\n", currLat, currLon, pathIndex, d);
            xSemaphoreGive(gpsDataMutex);
          }
          lastLat = currLat; lastLon = currLon;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200)); // 5Hz feed/poll
  }
}

// -------------------- Reverse navigation task --------------------
void ReverseNavTask(void* pvParameters) {
  for (;;) {
    // Only do reverse navigation if there's at least one saved point and targetIndex set
    if (pathIndex > 0) {
      // initialize targetIndex if not set
      if (targetIndex < 0) {
        if (xSemaphoreTake(gpsDataMutex, portMAX_DELAY) == pdTRUE) {
          targetIndex = pathIndex - 1;
          xSemaphoreGive(gpsDataMutex);
        }
      }

      // current GPS read (no gpsDataMutex required to read TinyGPS directly)
      if (!gps.location.isValid()) {
        vTaskDelay(pdMS_TO_TICKS(200));
        continue;
      }
      float currLat = gps.location.lat();
      float currLon = gps.location.lng();

      // copy target waypoint safely
      Coordinate tgt;
      if (xSemaphoreTake(gpsDataMutex, portMAX_DELAY) == pdTRUE) {
        if (targetIndex >= 0 && targetIndex < pathIndex) {
          tgt = path[targetIndex];
        } else {
          // no valid target
          xSemaphoreGive(gpsDataMutex);
          vTaskDelay(pdMS_TO_TICKS(200));
          continue;
        }
        xSemaphoreGive(gpsDataMutex);
      } else {
        vTaskDelay(pdMS_TO_TICKS(200));
        continue;
      }

      // compute distance & bearing
      float dist = distanceBetween(currLat, currLon, tgt.lat, tgt.lon);
      float bearing = calculateBearing(currLat, currLon, tgt.lat, tgt.lon);

      // read current heading under i2cMutex (avoid racing with CompassTask)
      float currHeadingLocal = heading;
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        currHeadingLocal = heading;
        xSemaphoreGive(i2cMutex);
      }

      // update globals (protected by gpsDataMutex)
      if (xSemaphoreTake(gpsDataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        navDistance = dist;
        navBearing  = bearing;
        navHeading  = currHeadingLocal;
        // if arrived at this waypoint, decrement targetIndex
        if (dist <= ARRIVE_DIST_METERS && targetIndex > 0) {
          targetIndex--;
        }
        xSemaphoreGive(gpsDataMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

// -------------------- Sensor helpers --------------------

void initSensors() {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    // QMC5883L soft reset
    Wire.beginTransmission(QMC5883L_ADDR);
    Wire.write(0x0B); Wire.write(0x01);
    Wire.endTransmission();
    delay(50);
    // continuous mode / register 0x09 configuration (as used previously)
    Wire.beginTransmission(QMC5883L_ADDR);
    Wire.write(0x09); Wire.write(0x1D);
    Wire.endTransmission();

    // Wake MPU6500
    Wire.beginTransmission(MPU6500_ADDR);
    Wire.write(0x6B); Wire.write(0x00);
    Wire.endTransmission();

    xSemaphoreGive(i2cMutex);
  } else {
    Serial.println("initSensors: couldn't take i2cMutex");
  }
}

// read 6 bytes from QMC5883L starting register 0x00 (little-endian)
void readMagnetometer(int16_t &mx_out, int16_t &my_out, int16_t &mz_out) {
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.requestFrom(QMC5883L_ADDR, (uint8_t)6);
  if (Wire.available() >= 6) {
    uint8_t lx = Wire.read();
    uint8_t hx = Wire.read();
    uint8_t ly = Wire.read();
    uint8_t hy = Wire.read();
    uint8_t lz = Wire.read();
    uint8_t hz = Wire.read();
    mx_out = (int16_t)((uint16_t)hx << 8 | (uint16_t)lx);
    my_out = (int16_t)((uint16_t)hy << 8 | (uint16_t)ly);
    mz_out = (int16_t)((uint16_t)hz << 8 | (uint16_t)lz);
  } else {
    // keep previous or set zero if needed
  }
}

// read accelerometer from MPU6500 ACCEL_XOUT_H (0x3B) (big-endian)
void readAccelerometer(int16_t &ax_out, int16_t &ay_out, int16_t &az_out) {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission();

  Wire.requestFrom(MPU6500_ADDR, (uint8_t)6);
  if (Wire.available() >= 6) {
    uint8_t axh = Wire.read(); uint8_t axl = Wire.read();
    uint8_t ayh = Wire.read(); uint8_t ayl = Wire.read();
    uint8_t azh = Wire.read(); uint8_t azl = Wire.read();
    ax_out = (int16_t)((uint16_t)axh << 8 | (uint16_t)axl);
    ay_out = (int16_t)((uint16_t)ayh << 8 | (uint16_t)ayl);
    az_out = (int16_t)((uint16_t)azh << 8 | (uint16_t)azl);
  } else {
    // keep previous
  }
}

// haversine distance (meters)
float distanceBetween(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371000.0f;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  lat1 = radians(lat1); lat2 = radians(lat2);
  float a = sin(dLat/2)*sin(dLat/2) + cos(lat1)*cos(lat2)*sin(dLon/2)*sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

// initial bearing from point1 to point2 in degrees (0..360)
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  float dLon = radians(lon2 - lon1);
  lat1 = radians(lat1); lat2 = radians(lat2);
  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dLon);
  float brng = atan2(y, x);
  brng = degrees(brng);
  return fmod(brng + 360.0f, 360.0f);
}