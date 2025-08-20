#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// QMC5883L and MPU6500 Addresses
#define QMC5883L_ADDR 0x0D
#define MPU6500_ADDR  0x68

int16_t mx, my, mz; // magnetometer
int16_t ax, ay, az; // accelerometer
float heading;

void setup() {
  Serial.begin(115200);
  Wire.begin(); // SDA=21, SCL=22 for ESP32

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED not found"));
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Compass Initializing...");
  display.display();
  delay(1000);

  // QMC5883L init
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x0B); Wire.write(0x01); // soft reset
  Wire.endTransmission();
  delay(100);

  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x09); Wire.write(0x1D); // Continuous, 200Hz, 2G, 128 OSR
  Wire.endTransmission();

  // MPU6500 init
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x6B); Wire.write(0x00); // Wake up MPU6500
  Wire.endTransmission();
}

void loop() {
  readMagnetometer();
  readAccelerometer();

  // Normalize accelerometer data
  float axn = (float)ax;
  float ayn = (float)ay;
  float azn = (float)az;
  float norm = sqrt(axn * axn + ayn * ayn + azn * azn);
  axn /= norm;
  ayn /= norm;
  azn /= norm;

  // Tilt Compensation
  float xh = mx * azn - mz * axn;
  float yh = my * azn - mz * ayn;
  heading = atan2(yh, xh) * 180 / PI;
  if (heading < 0) heading += 360;

  // Debug serial output
  Serial.print("Mag X: "); Serial.print(mx);
  Serial.print(" Y: "); Serial.print(my);
  Serial.print(" Z: "); Serial.print(mz);
  Serial.print(" | Acc X: "); Serial.print(ax);
  Serial.print(" Y: "); Serial.print(ay);
  Serial.print(" Z: "); Serial.print(az);
  Serial.print(" | Heading: "); Serial.println(heading);

  // Display on OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print("Heading: ");
  display.print(heading, 1);
  display.println(" deg");

  display.print("MX: "); display.print(mx);
  display.print(" MY: "); display.println(my);
  display.print("MZ: "); display.println(mz);

  // Compass Arrow Drawing
  int centerX = SCREEN_WIDTH / 2;
  int centerY = SCREEN_HEIGHT / 2 + 5;
  int arrowLength = 20;

  float angleRad = heading * PI / 180.0;

  int arrowX = centerX + arrowLength * sin(angleRad);
  int arrowY = centerY - arrowLength * cos(angleRad);  // Y axis reversed on screen

  display.drawCircle(centerX, centerY, 2, SSD1306_WHITE); // center dot
  display.drawLine(centerX, centerY, arrowX, arrowY, SSD1306_WHITE); // direction arrow

  display.setTextSize(2);
  display.setCursor(90, 45);
  if (heading >= 337.5 || heading < 22.5)       display.println("N");
  else if (heading >= 22.5 && heading < 67.5)   display.println("NE");
  else if (heading >= 67.5 && heading < 112.5)  display.println("E");
  else if (heading >= 112.5 && heading < 157.5) display.println("SE");
  else if (heading >= 157.5 && heading < 202.5) display.println("S");
  else if (heading >= 202.5 && heading < 247.5) display.println("SW");
  else if (heading >= 247.5 && heading < 292.5) display.println("W");
  else if (heading >= 292.5 && heading < 337.5) display.println("NW");

  display.display();
  delay(500);
}

void readMagnetometer() {
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(QMC5883L_ADDR, 6);
  if (Wire.available() == 6) {
    mx = Wire.read() | (Wire.read() << 8);
    my = Wire.read() | (Wire.read() << 8);
    mz = Wire.read() | (Wire.read() << 8);
  }
}

void readAccelerometer() {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x3B); // Start at ACCEL_XOUT_H
  Wire.endTransmission();
  Wire.requestFrom(MPU6500_ADDR, 6);
  if (Wire.available() == 6) {  
    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();
  }
}
