// This is the final program - dont change this.
#include "Protocol.h"
#include "BLEVectorSyncServer.h"
#include <Arduino.h>
#include <LoRa.h>
#include <U8g2lib.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <esp_sleep.h>
#include "driver/rtc_io.h"
#include "esp32/rom/uart.h"
UserDevicePayload userDevice(42);
std::vector<std::string> PredefinedMessages = userDevice.getPredefinedMessagesForUser();
UserDevicePayload::Coordinate currentDeviceGPS;
bool gpsFix = false;
// Storage
std::vector<std::string> *inboxBucketPtr;
std::vector<std::string> *sentBoxBucketPtr;
std::vector<std::string> *gpsStringBucketPtr;
std::vector<UserDevicePayload::Coordinate> *gpsBucket;
// BLE
BLEVectorSyncServer bleServer("LORA 10 User Device");
BLEVectorSync *gpsSync;
BLEVectorSync *inboxSync;
BLEVectorSync *sendboxSync;
void handleMessageFromBLE(const std::string &msg);

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;
TaskHandle_t Task5;
TaskHandle_t Task6;
// TaskHandle_t SleepManagerTaskHandle;

void _LoRaListenTask(void *pvParameters);
void _ButtonPressTask(void *pvParameters);
void _OledDisplayTask(void *pvParameters);
void _GpsUpdateTask(void *pvParameters);
void _LoRaSendTask(void *pvParameters);
void _BackNavigationTask(void *pvParameters);

// Pins
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 26
int RSSI = -50;

//------BUTTONS---------
#define OK_BTN 32
#define UP_BTN 25
#define DOWN_BTN 33
#define MODE_BTN 27
#define SLEEP_BTN 2

void modeBtnPressed();
void sleepBtnPressed();
void upBtnPressed();
void downBtnPressed();
void okBtnPressed();

struct ButtonConfig
{
    uint8_t pin;
    const char *name;
    uint32_t debounceMs;
    void (*onPress)();
    bool lastStable;
    uint32_t lastChangeMs;
};

ButtonConfig buttons[] = {
    {MODE_BTN, "MODE", 50, modeBtnPressed, false, 0}, // long debounce
    {UP_BTN, "UP", 50, upBtnPressed, false, 0},
    {DOWN_BTN, "DOWN", 50, downBtnPressed, false, 0},
    {OK_BTN, "OK", 50, okBtnPressed, false, 0},
    {SLEEP_BTN, "SLEEP", 50, sleepBtnPressed, false, 0} // long debounce
};

const size_t BUTTON_COUNT = sizeof(buttons) / sizeof(buttons[0]);

int page = 0;
int states[5][2] = {0};

SemaphoreHandle_t xSemaphore;
SemaphoreHandle_t i2cMutex;
QueueHandle_t loraQueue;
//----------------------
#define QMC5883L_ADDR 0x0D // magnetometer QMC5883L typical address
#define MPU6500_ADDR 0x68  // MPU6500/MPU6050
void initAccelerometerAndMagnetometer();
void readMagnetometer(int16_t &x, int16_t &y, int16_t &z);
void readAccelerometer(int16_t &x, int16_t &y, int16_t &z);
float distanceBetween(float lat1, float lon1, float lat2, float lon2);
float calculateBearing(float lat1, float lon1, float lat2, float lon2);
static const int MAX_WAYPOINTS = 100;
static const float SAVE_DIST_METERS = 5.0f;
static const float ARRIVE_DIST_METERS = 5.0f;
int pathIndex = 0;
int targetIndex = -1;
int startp = 2;
volatile float heading = 0.0f;        // updated by CompassTask (deg)
volatile float navDistance = 0.0f;    // distance to current target (m)
volatile float navBearing = 0.0f;     // bearing to current target (deg)
volatile float navHeading = 0.0f;     // current compass heading (deg)
volatile float compassHeading = 0.0f; // degrees 0..360, written by _CompassTask, read by renderCompass()

// OLED
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
U8G2_SSD1309_128X128_NONAME0_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE); // fore Big one
// U8G2_SH1107_128X128_F_HW_I2C u8g2( U8G2_R0,U8X8_PIN_NONE);
// U8G2_SH1107_SEEED_128X128_F_HW_I2C u8g2(
//     U8G2_R0,
//     U8X8_PIN_NONE,
//     /* scl=*/22,
//     /* sda=*/21);

#define H_FONT u8g2_font_ncenB08_tr
#define P_FONT u8g2_font_6x10_tr

#define im_width 128
#define im_height 48
static const uint8_t im_bits[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x00, 0x80, 0xff,
                                  0xff, 0xcf, 0xff, 0xff, 0xe7, 0xff, 0xff, 0x07, 0xf8, 0xf3, 0xff, 0xff,
                                  0x7f, 0x00, 0x80, 0xff, 0xff, 0xcf, 0xff, 0xff, 0xe7, 0xff, 0xff, 0x07,
                                  0xf8, 0xf3, 0xff, 0xff, 0x7f, 0x00, 0x80, 0xff, 0xff, 0xcf, 0xff, 0xff,
                                  0xe7, 0xff, 0xff, 0x07, 0xf8, 0xf3, 0xff, 0xff, 0x7f, 0x00, 0x80, 0xff,
                                  0xff, 0xcf, 0xff, 0xff, 0xe7, 0xff, 0xff, 0x07, 0xf8, 0xf3, 0xff, 0xff,
                                  0x7f, 0x00, 0x80, 0xff, 0xff, 0xcf, 0xff, 0xff, 0xe7, 0xff, 0xff, 0x07,
                                  0xf8, 0xf3, 0xff, 0xff, 0x7f, 0x00, 0x80, 0xff, 0xff, 0xcf, 0xff, 0xff,
                                  0xe7, 0xff, 0xff, 0x07, 0xf8, 0xf3, 0xff, 0xff, 0x7f, 0x00, 0x80, 0xff,
                                  0xff, 0xcf, 0xff, 0xff, 0xe7, 0xff, 0xff, 0x07, 0xf8, 0xf3, 0xff, 0xff,
                                  0x7f, 0x00, 0x80, 0x3f, 0xe0, 0xcf, 0x3f, 0xf8, 0xe7, 0x1f, 0xf8, 0x07,
                                  0xf8, 0xf3, 0xff, 0xff, 0x7f, 0x00, 0x80, 0x3f, 0xe0, 0xcf, 0x3f, 0xfc,
                                  0xe7, 0x1f, 0xf8, 0x07, 0xf8, 0xf3, 0xff, 0xff, 0x7f, 0x00, 0x80, 0x3f,
                                  0xe0, 0xcf, 0x3f, 0xfe, 0xe7, 0xff, 0xff, 0x07, 0xf8, 0xf3, 0x9f, 0xff,
                                  0x7f, 0x00, 0x80, 0x3f, 0xe0, 0xcf, 0x3f, 0xff, 0xe3, 0xff, 0xff, 0x07,
                                  0xf8, 0xf3, 0x1f, 0xff, 0x7f, 0x00, 0x80, 0x3f, 0xe0, 0xcf, 0xbf, 0xff,
                                  0xe1, 0xff, 0xff, 0x07, 0xf8, 0xf3, 0x0f, 0xff, 0x7f, 0x00, 0x80, 0x3f,
                                  0xe0, 0xcf, 0xdf, 0xff, 0xe0, 0xff, 0xff, 0x07, 0xf8, 0xf3, 0x07, 0xfe,
                                  0x7f, 0x00, 0x80, 0x3f, 0xe0, 0xcf, 0xbf, 0xff, 0xe1, 0xff, 0xff, 0x07,
                                  0xf8, 0xf3, 0x03, 0xfc, 0x7f, 0x00, 0x80, 0x3f, 0xe0, 0xcf, 0x3f, 0xff,
                                  0xe3, 0xff, 0xff, 0x07, 0xf8, 0xf3, 0x03, 0xfc, 0x7f, 0x00, 0x80, 0x3f,
                                  0xe0, 0xcf, 0x3f, 0xfe, 0xe7, 0xff, 0xff, 0x07, 0xf8, 0xf3, 0x01, 0xf8,
                                  0x7f, 0x00, 0x80, 0x3f, 0xe0, 0xcf, 0x3f, 0xfc, 0xe7, 0x1f, 0xf8, 0x07,
                                  0xf8, 0xf3, 0xff, 0xff, 0x7f, 0x00, 0x80, 0x3f, 0xe0, 0xcf, 0x3f, 0xf8,
                                  0xe7, 0x1f, 0xf8, 0x07, 0xf8, 0xf3, 0xff, 0xff, 0xff, 0xff, 0x9f, 0xff,
                                  0xff, 0xcf, 0x3f, 0xf0, 0xe7, 0x1f, 0xf8, 0x07, 0xf8, 0xf3, 0xff, 0xff,
                                  0xff, 0xff, 0x9f, 0xff, 0xff, 0xcf, 0x3f, 0xe0, 0xe7, 0x1f, 0xf8, 0x07,
                                  0xf8, 0xf3, 0xff, 0xff, 0xff, 0xff, 0x9f, 0xff, 0xff, 0xcf, 0x3f, 0xc0,
                                  0xe7, 0x1f, 0xf8, 0x07, 0xf8, 0xf3, 0xff, 0xff, 0xff, 0xff, 0x9f, 0xff,
                                  0xff, 0xcf, 0x3f, 0x80, 0xe7, 0x1f, 0xf8, 0x07, 0xf8, 0xf3, 0xff, 0xff,
                                  0xff, 0xff, 0x9f, 0xff, 0xff, 0xcf, 0x3f, 0x00, 0xe7, 0x1f, 0xf8, 0x07,
                                  0xf8, 0xf3, 0xff, 0xff, 0xff, 0xff, 0x9f, 0xff, 0xff, 0xcf, 0x3f, 0x00,
                                  0xe6, 0x1f, 0xf8, 0x07, 0xf8, 0xf3, 0xff, 0xff, 0xff, 0xff, 0x9f, 0xff,
                                  0xff, 0xcf, 0x3f, 0x00, 0xe4, 0x1f, 0xf8, 0x07, 0xf8, 0xf3, 0xff, 0xff,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// GPS
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
HardwareSerial GPS_Serial(2);
TinyGPSPlus gps;
SemaphoreHandle_t gpsMutex;

#define BUZZER_PIN 4
#define PWM_CHANNEL 0
#define PWM_FREQ 2000
#define PWM_RESOLUTION 8
void setupBuzzer();
void beep();
void beep2();
void beep3();
void noTone();

// Battry
#define BATTERY_PIN 34
const float R1 = 100000.0;     // 100k
const float R2 = 47000.0;      // 47k
const float MAX_VOLTAGE = 8.4; // Fullv
const float MIN_VOLTAGE = 6.0; // Empty
float readBatteryVoltage();

void setup()
{
    Serial.begin(115200);
    GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("[*] Starting Device... [Serial:115200]");
    gpsBucket = userDevice.getGpsBucket();
    gpsStringBucketPtr = userDevice.getGpsStringBucket();
    gpsSync = new BLEVectorSync("gps", *gpsStringBucketPtr);
    inboxBucketPtr = userDevice.getInboxBucket();
    inboxSync = new BLEVectorSync("inbox", *inboxBucketPtr);
    sentBoxBucketPtr = userDevice.getSentboxBucket();
    sendboxSync = new BLEVectorSync("sendbox", *sentBoxBucketPtr);
    bleServer.addVector(inboxSync);
    bleServer.addVector(sendboxSync);
    bleServer.addVector(gpsSync);
    bleServer.setDataCallback(handleMessageFromBLE);
    while (!bleServer.begin(120, ESP_PWR_LVL_P9))
    {
        Serial.println("Failed to start BLE server, retrying...");
        delay(1000);
    }
    u8g2.begin();
    u8g2.clearBuffer();
    setupBuzzer();
    // pinMode(RED_LED, OUTPUT);
    // pinMode(BLUE_LED, OUTPUT);
    // digitalWrite(RED_LED, HIGH);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    Serial.println("[+] LEDs initialized.");

    for (size_t i = 0; i < BUTTON_COUNT; i++)
    {
        pinMode(buttons[i].pin, INPUT_PULLDOWN);

        // external pulldown present
        buttons[i].lastStable = (digitalRead(buttons[i].pin) == HIGH);
        buttons[i].lastChangeMs = millis();
    }
    rtc_gpio_pulldown_en((gpio_num_t)SLEEP_BTN);
    rtc_gpio_pullup_dis((gpio_num_t)SLEEP_BTN);
    Serial.println("[+] Button handlers initialized.");

    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if (!LoRa.begin(433E6))
    {
        Serial.println("LoRa init failed.");
        while (1)
            ;
    }
    LoRa.setFrequency(433E6); // or 868E6 / 915E6 depending on module & region
    // LoRa.setTxPower(18);                         // Max for SX1278 (in dBm) — use 17–20
    LoRa.setSpreadingFactor(13);     // 6–12 (12 = max range, min speed)
    LoRa.setSignalBandwidth(62.5E3); // 7.8kHz–500kHz (62.5kHz = good compromise)
    // LoRa.setCodingRate4(8);                      // 4/5–4/8 (4/8 = strongest error correction)
    LoRa.enableCrc();
    // ensure CRC check
    rtc_gpio_pulldown_en((gpio_num_t)LORA_DIO0); // Stabilize DIO0
    rtc_gpio_pullup_dis((gpio_num_t)LORA_DIO0);
    LoRa.receive();
    // LoRa.setSyncWord(0x12);
    // LoRa.setTxPower(15);
    // LoRa.setSpreadingFactor(13);
    Serial.println("[+] LoRa init succeeded.");

    i2cMutex = xSemaphoreCreateMutex();
    initAccelerometerAndMagnetometer();

    Serial.println("[+] Accelerometer and Magnetometer initialized.");
    xSemaphore = xSemaphoreCreateMutex();
    gpsMutex = xSemaphoreCreateMutex();
    loraQueue = xQueueCreate(10, sizeof(std::vector<int8_t> *));

    // core 0 system , core 1 user
    xTaskCreatePinnedToCore(_LoRaListenTask, "LoRaListenTask", 8192, NULL, 3, &Task5, 0);
    xTaskCreatePinnedToCore(_LoRaSendTask, "LoRaSendTask", 2048, NULL, 1, &Task1, 0);
    xTaskCreatePinnedToCore(_GpsUpdateTask, "GpsUpdateTask", 4096, NULL, 1, &Task4, 1);
    xTaskCreatePinnedToCore(_ButtonPressTask, "ButtonPressTask", 2048, NULL, 2, &Task2, 1);
    xTaskCreatePinnedToCore(_OledDisplayTask, "OledDisplayTask", 4096, NULL, 1, &Task3, 1);
    xTaskCreatePinnedToCore(_BackNavigationTask, "BackNavigationTask", 4096, NULL, 2, &Task6, 0);

    // xTaskCreatePinnedToCore(_SleepManagerTask, "SleepManagerTask", 4096, NULL, 2, &SleepManagerTaskHandle, 1);

    // digitalWrite(BLUE_LED, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    beep3();
    // digitalWrite(RED_LED, LOW);
    // digitalWrite(BLUE_LED, LOW);
}

void loop()
{
    // Not needed as FreeRTOS manages tasks
}

// BTN
void modeBtnPressed()
{
    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(200)))
    {
        if (page == 1 && states[1][1] == 1) // Inbox detail view
        {
            states[1][1] = 0; // Go back to list view
        }
        if (page == 3 && (states[3][1] == 1 || states[3][1] == 2))
        {
            // Return to selection prompt when leaving page 3
            states[3][1] = 0; // 0 = show selection prompt next time
            states[3][0] = 0; // optional: reset cursor to "Compass"
        }
        if (page == 4 && states[4][1] == 1) // SentBox detail view
        {
            states[4][1] = 0; // Go back to list view
        }
        page = (page + 1) % 5; // Cycle through screens 0..4
        xSemaphoreGive(xSemaphore);
    }
}

void sleepBtnPressed()
{
    Serial.println("[SleepManager] Sleep button pressed → Preparing for Light Sleep...");

    // Wait for button release
    while (digitalRead(SLEEP_BTN) == HIGH)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Clear and power down OLED
    u8g2.clearBuffer();
    u8g2.sendBuffer();
    u8g2.setPowerSave(1); // Turn off OLED display

    // Stop BLE to allow light sleep
    // bleServer.stop();
    // Serial.println("[SleepManager] BLE stopped for sleep");

    // Ensure LoRa is in continuous receive
    LoRa.receive();

    // Ensure Serial output completes
    uart_tx_wait_idle(CONFIG_ESP_CONSOLE_UART_NUM);

    // Enable ext0 wake on SLEEP_BTN (HIGH) and ext1 for DIO0 (HIGH)
    esp_sleep_enable_ext0_wakeup((gpio_num_t)SLEEP_BTN, 1);
    esp_sleep_enable_ext1_wakeup((1ULL << LORA_DIO0), ESP_EXT1_WAKEUP_ANY_HIGH);

    // Enter light sleep
    esp_light_sleep_start();

    // Resume after wake-up
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0)
    {
        Serial.println("[SleepManager] Woke up from Light Sleep! Cause = EXT0 (Button press)");
        while (digitalRead(SLEEP_BTN) == HIGH)
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
    else if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1)
    {
        Serial.println("[SleepManager] Woke up from Light Sleep! Cause = EXT1 (LoRa DIO0)");
        // Process LoRa packet
        int packetSize = LoRa.parsePacket();
        if (packetSize)
        {
            std::vector<int8_t> newReceivedPayload;
            RSSI = LoRa.packetRssi();
            while (LoRa.available())
            {
                newReceivedPayload.push_back(LoRa.read());
            }
            userDevice.receive(newReceivedPayload);
            Serial.println("[Payload Received]");
            // Serial.println(userDevice.getJsonPayload().c_str());
        }
    }
    else
    {
        Serial.printf("[SleepManager] Woke up from Light Sleep! Cause = %d\n", wakeup_reason);
    }

    // Restore OLED and BLE
    u8g2.setPowerSave(0); // Turn OLED back on
    u8g2.clearBuffer();
    u8g2.sendBuffer();
    // while (!bleServer.begin(120, ESP_PWR_LVL_P9)) {
    //     Serial.println("[SleepManager] Retrying BLE start...");
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    // Serial.println("[SleepManager] BLE and OLED restored");
}

void upBtnPressed()
{
    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(200)))
    {
        if (page == 1 && states[1][0] > 0) // Inbox
        {
            states[1][0]--;
        }
        else if (page == 2 && states[2][0] > 0) // Send
        {
            states[2][0]--;
        }
        else if (page == 3 && states[3][0] >= 0 && states[3][0] <= 1) // Compass
        {
            states[3][0] = (states[3][0] + 1) % 2;
        }
        else if (page == 4 && states[4][0] > 0) // SentBox
        {
            states[4][0]--;
        }
        xSemaphoreGive(xSemaphore);
    }
}

void downBtnPressed()
{
    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(200)))
    {
        int sizeofinbox = inboxBucketPtr->size();
        // printf("Inbox size: %d\n", sizeofinbox);
        if (page == 1 && states[1][0] + 1 < inboxBucketPtr->size()) // Inbox
        {
            states[1][0]++;
        }
        else if (page == 2 && states[2][0] + 1 < PredefinedMessages.size()) // Send
        {
            states[2][0]++;
        }
        else if (page == 3 && states[3][0] >= 0 && states[3][0] <= 1) // Compass
        {
            states[3][0] = (states[3][0] + 1) % 2;
        }
        else if (page == 4 && states[4][0] + 1 < sentBoxBucketPtr->size()) // SentBox
        {
            states[4][0]++;
        }
        xSemaphoreGive(xSemaphore);
    }
}

void okBtnPressed()
{
    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(200)))
    {
        if (page == 1) // Inbox
        {
            if (states[1][1] == 0)
            {
                // Switch from list view -> detail view of selected message
                states[1][1] = 1;
            }
            else
            {
                // Already inside a message -> go back to list
                states[1][1] = 0;
            }
        }
        else if (page == 2) // Send
        {
            states[2][1] = 1;
            userDevice.createPmsg(states[2][0], 0); // Create predefined message
            std::vector<int8_t> *payloadVector = new std::vector<int8_t>(userDevice.getPayload());
            xQueueSend(loraQueue, &payloadVector, portMAX_DELAY); // send selection
        }
        else if (page == 3) // Compass
        {
            if (states[3][0] == 1)
            {
                states[3][1] = 1; // Set detail flag
            }
            else
            {
                states[3][1] = 2; // Reset detail flag
            }
        }
        else if (page == 4) // SentBox
        {
            if (states[4][1] == 0)
            {
                // Switch from list view -> detail view of selected message
                states[4][1] = 1;
            }
            else
            {
                // Already inside a message -> go back to list
                states[4][1] = 0;
            }
        }

        xSemaphoreGive(xSemaphore);
    }
}

void printStatesOfButtons()
{
    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5)))
    {
        Serial.print("Page: ");
        Serial.println(page);
        Serial.println("States:");
        for (int i = 0; i < 5; i++)
        {
            Serial.print("State[");
            Serial.print(i);
            Serial.print("][0]: ");
            Serial.print(states[i][0]);
            Serial.print(" | State[");
            Serial.print(i);
            Serial.print("][1]: ");
            Serial.println(states[i][1]);
        }
        xSemaphoreGive(xSemaphore);
    }
}

void _ButtonPressTask(void *pvParameters)
{
    const TickType_t scanDelay = pdMS_TO_TICKS(5); // check every 5ms
    while (true)
    {
        uint32_t now = millis();

        for (size_t i = 0; i < BUTTON_COUNT; i++)
        {
            bool read = (digitalRead(buttons[i].pin) == HIGH);

            if (read != buttons[i].lastStable)
            {
                // only update if stable long enough
                if ((now - buttons[i].lastChangeMs) >= buttons[i].debounceMs)
                {
                    buttons[i].lastStable = read;
                    buttons[i].lastChangeMs = now;

                    if (read == HIGH)
                    { // Press event
                        Serial.println(buttons[i].name);
                        buttons[i].onPress();
                        beep();
                    }
                }
            }
            else
            {
                // if same state, just update timestamp
                buttons[i].lastChangeMs = now;
            }
        }

        vTaskDelay(scanDelay);
    }
}

// OLED

void renderWelcome()
{
    const int LOGO_HEIGHT = 48;

    const int margin = 8;
    const int content_x = margin;
    const int content_width = 128 - 2 * margin;

    const int bx = margin;
    const int by = LOGO_HEIGHT + 8;
    const int bw = 36;
    const int bh = 18;
    const int tip_w = 4;
    const int tip_h = bh / 2;

    const int text_x = bx + bw + tip_w + 8;
    const int pct_y = by + (bh / 2) + 4;
    const int first_line_y = pct_y;
    const int rssi_y = by + bh + 16;
    const int gps_y = rssi_y + 16;

    float batteryVoltage = readBatteryVoltage();
    int percentage = round(((batteryVoltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100);
    if (percentage > 100)
        percentage = 100;
    if (percentage < 0)
        percentage = 0;

    u8g2.firstPage();
    do
    {
        // --- LOGO AREA ---
        // Draw the logo at (0,0)
        u8g2.drawXBMP(0, 0, im_width, im_height, im_bits);
        u8g2.drawLine(0, im_height - 2, 127, im_height - 2);
        u8g2.drawLine(0, im_height - 1, 127, im_height - 1);

        // --- Battery icon (small) ---
        u8g2.drawFrame(bx, by, bw, bh);
        u8g2.drawBox(bx + bw, by + (bh - tip_h) / 2, tip_w, tip_h);

        int inner_x = bx + 2;
        int inner_w = bw - 4;
        int fillWidth = (inner_w * percentage) / 100;
        if (fillWidth > 0)
        {
            if (percentage <= 20)
            {
                u8g2.setDrawColor(0);
                u8g2.drawBox(inner_x, by + 2, fillWidth, bh - 4);
                u8g2.setDrawColor(1);
            }
            else
            {
                u8g2.drawBox(inner_x, by + 2, fillWidth, bh - 4);
            }
        }

        char percentBuf[8];
        sprintf(percentBuf, "%d%%", percentage);
        u8g2.setFont(H_FONT);
        u8g2.drawStr(text_x, pct_y, percentBuf);

        char rssiBuf[20];
        sprintf(rssiBuf, "RSSI: %d", RSSI);
        u8g2.setFont(P_FONT);
        u8g2.drawStr(content_x, rssi_y, rssiBuf);

        const char *gpsStr = gpsFix ? "GPS: Active" : "GPS: Waiting...";
        u8g2.drawStr(content_x, gps_y, gpsStr);
        u8g2.setFont(H_FONT);
        int8_t did = userDevice.getDid();
        char didBuf[32];
        sprintf(didBuf, "- User Device : %d -", did);
        int didWidth = u8g2.getStrWidth(didBuf);
        int didX = (128 - didWidth) / 2;
        u8g2.drawStr(didX, gps_y + 16, didBuf);

    } while (u8g2.nextPage());
}
// void renderWelcome()
// {
//     u8g2.firstPage();
//     do
//     {
//         float batteryVoltage = readBatteryVoltage();

//         // Calculate battery percentage
//         int percentage = round(((batteryVoltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100);
//         if (percentage > 100)
//             percentage = 100;
//         if (percentage < 0)
//             percentage = 0;

//         u8g2.clearBuffer();

//         // Header
//         u8g2.setFont(H_FONT);
//         u8g2.drawStr(32, 18, "LORA10 DEVICE");
//         u8g2.drawLine(10, 22, 118, 22);

//         // RSSI (signal strength) - top right
//         char rssiBuf[16];
//         sprintf(rssiBuf, "RSSI: %d", RSSI);
//         u8g2.setFont(P_FONT);
//         u8g2.drawStr(80, 12, rssiBuf);

//         // GPS Status - top left
//         const char *gpsStr = gpsFix ? "GPS: Active" : "GPS: Waiting...";
//         u8g2.drawStr(10, 12, gpsStr);

//         // Battery Icon (centered, large)
//         int bx = 34, by = 38, bw = 60, bh = 28;
//         u8g2.drawFrame(bx, by, bw, bh);            // battery body
//         u8g2.drawBox(bx + bw, by + 8, 6, bh - 16); // battery tip

//         // Fill battery level
//         int fillWidth = (bw - 4) * percentage / 100;
//         int fillColor = percentage > 20 ? 1 : 0; // red if low (simulate by invert)
//         if (fillColor == 0)
//         {
//             u8g2.setDrawColor(0);
//         }
//         u8g2.drawBox(bx + 2, by + 2, fillWidth, bh - 4);
//         u8g2.setDrawColor(1);

//         // Battery % text inside battery
//         char percentBuf[8];
//         sprintf(percentBuf, "%d%%", percentage);
//         int percentWidth = u8g2.getStrWidth(percentBuf);
//         u8g2.setFont(H_FONT);
//         u8g2.drawStr(bx + (bw - percentWidth) / 2, by + bh - 8, percentBuf);

//         // Battery voltage below battery
//         char voltBuf[16];
//         sprintf(voltBuf, "%.2fV", batteryVoltage);
//         int voltWidth = u8g2.getStrWidth(voltBuf);
//         u8g2.setFont(P_FONT);
//         u8g2.drawStr(bx + (bw - voltWidth) / 2, by + bh + 12, voltBuf);

//         // Footer
//         u8g2.setFont(P_FONT);
//         u8g2.drawLine(10, 120, 118, 120);
//         u8g2.drawStr(32, 127, "Press MODE to start");

//         u8g2.sendBuffer();

//     } while (u8g2.nextPage());
// }

void renderInbox()
{
    const int VISIBLE = 11;      // number of lines shown on screen at once
    static int firstVisible = 0; // preserved between calls

    u8g2.firstPage();
    do
    {
        u8g2.clearBuffer();
        u8g2.setFont(H_FONT);
        u8g2.drawStr(startp, 10, "INBOX");
        u8g2.drawLine(0, 11, 127, 11);

        u8g2.setFont(P_FONT);
        int total = (int)inboxBucketPtr->size();

        if (total == 0)
        {
            u8g2.drawStr(startp, 25, "No messages");
        }
        else
        {
            // clamp selected index
            if (states[1][0] < 0)
                states[1][0] = 0;
            if (states[1][0] >= total)
                states[1][0] = total - 1;

            // ensure firstVisible is valid for current selection
            if (states[1][0] < firstVisible)
            {
                firstVisible = states[1][0];
            }
            else if (states[1][0] >= firstVisible + VISIBLE)
            {
                firstVisible = states[1][0] - VISIBLE + 1;
            }

            // clamp firstVisible bounds
            if (firstVisible < 0)
                firstVisible = 0;
            if (firstVisible > total - VISIBLE)
            {
                firstVisible = max(0, total - VISIBLE);
            }

            // draw visible messages (max VISIBLE)
            for (int row = 0; row < VISIBLE; row++)
            {
                int idx = firstVisible + row;
                if (idx >= total)
                    break;
                int y = 25 + row * 10;

                if (idx == states[1][0])
                {
                    u8g2.drawBox(0, y - 8, 128, 10);
                    u8g2.setDrawColor(0);
                    u8g2.drawStr(2, y, inboxBucketPtr->at(idx).c_str());
                    u8g2.setDrawColor(1);
                }
                else
                {
                    u8g2.drawStr(2, y, inboxBucketPtr->at(idx).c_str());
                }
            }

            if (states[1][1])
            {
                // ensure selected index is valid
                int sel = states[1][0];
                int total = (int)inboxBucketPtr->size();
                if (total <= 0)
                {
                    // nothing to show
                }
                else
                {
                    if (sel < 0)
                        sel = 0;
                    if (sel >= total)
                        sel = total - 1;

                    // clear the current buffer and draw only the selected message
                    u8g2.clearBuffer();
                    u8g2.setFont(P_FONT);
                    u8g2.drawStr(startp, 10, "INBOX MESSAGE"); // header

                    std::string msg = inboxBucketPtr->at(sel);
                    const int maxCharsPerLine = 16; // tune to match your font/width
                    const int yStart = 25;
                    const int lineSpacing = 10;

                    // Draw message wrapped into fixed-width chunks
                    for (size_t pos = 0, line = 0; pos < msg.length(); pos += maxCharsPerLine, ++line)
                    {
                        std::string part = msg.substr(pos, maxCharsPerLine);
                        int y = yStart + line * lineSpacing;
                        if (y > 54)
                            break; // avoid overwriting footer area
                        u8g2.drawStr(startp, y, part.c_str());
                    }

                    // // footer: OK → Back
                    // u8g2.drawBox(0, 58, 128, 6);
                    // u8g2.setDrawColor(0);
                    // u8g2.drawStr(startp, 63, "Press OK");
                    // u8g2.setDrawColor(1);

                    // send this buffer for immediate display
                    u8g2.sendBuffer();
                }
            }
        }
    } while (u8g2.nextPage());
}

void renderSend()
{
    const int VISIBLE = 11;      // number of lines shown on screen at once
    static int firstVisible = 0; // preserved between calls

    u8g2.firstPage();
    do
    {
        u8g2.clearBuffer();
        u8g2.setFont(H_FONT);
        u8g2.drawStr(startp, 10, "SEND");
        u8g2.drawLine(0, 11, 127, 11);

        u8g2.setFont(P_FONT);
        int total = (int)PredefinedMessages.size();

        if (total == 0)
        {
            u8g2.drawStr(0, 25, "No messages");
            // also clear possible sending flag area
        }
        else
        {
            // clamp selected index
            if (states[2][0] < 0)
                states[2][0] = 0;
            if (states[2][0] >= total)
                states[2][0] = total - 1;

            // ensure firstVisible is valid for current selection
            if (states[2][0] < firstVisible)
            {
                firstVisible = states[2][0];
            }
            else if (states[2][0] >= firstVisible + VISIBLE)
            {
                firstVisible = states[2][0] - VISIBLE + 1;
            }

            // clamp firstVisible bounds
            if (firstVisible < 0)
                firstVisible = 0;
            if (firstVisible > total - VISIBLE)
            {
                firstVisible = max(0, total - VISIBLE);
            }

            // // optionally draw up/down arrows if there are more items
            // if (firstVisible > 0) {
            //     // simple up indicator (small triangle)
            //     int ux = 122, uy = 14;
            //     //u8g2.drawTriangle(ux, uy-2, ux-6, uy+4, ux+6, uy+4);
            // }
            // if (firstVisible + VISIBLE < total) {
            //     // simple down indicator
            //     int dx = 122, dy = 52;
            //     //u8g2.drawTriangle(dx, dy+2, dx-6, dy-6, dx+6, dy-6);
            // }

            // draw visible messages (max VISIBLE)
            for (int row = 0; row < VISIBLE; row++)
            {
                int idx = firstVisible + row;
                if (idx >= total)
                    break;
                int y = 25 + row * 10;

                if (idx == states[2][0])
                {
                    // highlight selected row
                    u8g2.drawBox(0, y - 8, 128, 10);
                    u8g2.setDrawColor(0); // invert color for text
                    u8g2.drawStr(startp, y, PredefinedMessages[idx].c_str());
                    u8g2.setDrawColor(1); // restore
                }
                else
                {
                    // normal text
                    u8g2.drawStr(startp, y, PredefinedMessages[idx].c_str());
                }
            }

            // Show detail-flag if OK pressed (sending)
            if (states[2][1])
            {
                u8g2.clearBuffer();
                // u8g2.drawBox(0, 58, 128, 6);
                // u8g2.setDrawColor(0);
                u8g2.drawStr(30, 32, "Sending...");
                // u8g2.setDrawColor(1);
                u8g2.sendBuffer();
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
    } while (u8g2.nextPage());
}

void renderCompass()
{
    u8g2.firstPage();
    do
    {
        // --- Selection prompt ---
        if (states[3][1] == 0)
        {
            u8g2.setFont(H_FONT);
            u8g2.drawStr(startp, 16, "NAVIGATION");
            u8g2.drawLine(0, 18, 127, 18);

            // Option 0: "Compass"
            if (states[3][0] == 0)
            {
                u8g2.drawBox(0, 30, 128, 14); // highlight box
                u8g2.setDrawColor(0);
                u8g2.drawStr(6, 40, "Compass");
                u8g2.setDrawColor(1);
            }
            else
            {
                u8g2.drawStr(6, 40, "Compass");
            }

            // Option 1: "Back Navigation"
            if (states[3][0] == 1)
            {
                u8g2.drawBox(0, 46, 128, 14); // highlight box
                u8g2.setDrawColor(0);
                u8g2.drawStr(6, 56, "Back Navigation");
                u8g2.setDrawColor(1);
            }
            else
            {
                u8g2.drawStr(6, 56, "Back Navigation");
            }
        }
        // --- Reverse (back) navigation mode ---

        else if (states[3][1] == 1)
        {
            float localBearing = 0.0f, localHeading = 0.0f, localDist = 0.0f;

            if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                localBearing = navBearing;
                localHeading = navHeading;
                localDist = navDistance;
                xSemaphoreGive(gpsMutex);
            }

            // Center the compass in a 128x128 display
            const int cx = 64;          // center X
            const int cy = 64;          // center Y (middle of 128)
            const int radius = 36;      // circle radius (adjust if you want larger/smaller)
            const int len = radius - 8; // needle length (keeps needle inside circle)

            u8g2.setFont(H_FONT);
            u8g2.drawStr(startp, 16, "BACK NAVIGATION");
            u8g2.drawLine(0, 18, 127, 18);

            // Draw compass circle and center dot
            u8g2.drawCircle(cx, cy, radius, 1); // outer circle
            u8g2.drawCircle(cx, cy, 2, 1);      // small center point

            // Compute normalized angle (bearing relative to heading)
            float angleDeg = localBearing - localHeading;
            while (angleDeg < 0.0f)
                angleDeg += 360.0f;
            while (angleDeg >= 360.0f)
                angleDeg -= 360.0f;
            float angleRad = radians(angleDeg);

            // Needle end point (sin/cos: X to right, Y downwards so subtract for screen coords)
            int ax = cx + (int)(len * sin(angleRad));
            int ay = cy - (int)(len * cos(angleRad));

            // Draw the needle
            u8g2.drawLine(cx, cy, ax, ay);

            // (Optional) draw a short tail for the needle so direction is clearer
            int tailLen = len / 4;
            int bx = cx - (int)(tailLen * sin(angleRad));
            int by = cy + (int)(tailLen * cos(angleRad));
            u8g2.drawLine(cx, cy, bx, by);

            // Distance text centered under the compass
            u8g2.setFont(P_FONT);
            char distBuf[32];
            snprintf(distBuf, sizeof(distBuf), "Dist: %.1f m", localDist);

            int textW = u8g2.getStrWidth(distBuf); // string pixel width
            int textH = u8g2.getMaxCharHeight();   // font height
            int textX = cx - (textW / 2);
            int textY = cy + radius + 6 + textH; // a little padding below circle

            // Ensure we don't write off-screen (clamp)
            if (textY > 127)
                textY = 127;

            u8g2.setCursor(textX, textY);
            u8g2.print(distBuf);
        }

        // --- Compass mode (normal compass) ---
        else if (states[3][1] == 2)
        {
            // Read the latest heading safely (degrees 0..360)
            float localHeadingDeg = 0.0f;
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                localHeadingDeg = compassHeading;
                xSemaphoreGive(i2cMutex);
            }
            else
            {
                localHeadingDeg = compassHeading; // fallback, small staleness acceptable
            }

            // Normalise to [0,360)
            if (!isfinite(localHeadingDeg))
                localHeadingDeg = 0.0f;
            while (localHeadingDeg < 0.0f)
                localHeadingDeg += 360.0f;
            while (localHeadingDeg >= 360.0f)
                localHeadingDeg -= 360.0f;

            // Draw compass face centered in the 128x128 display
            const int cx = 64;     // center x (middle of 128)
            const int cy = 64;     // center y (middle of 128)
            const int radius = 36; // larger radius to use more of the screen
            const int tickOut = 4; // tick length
            const int arrowLen = radius - 6;

            u8g2.setFont(H_FONT);
            u8g2.drawStr(startp, 16, "COMPASS");
            u8g2.drawLine(0, 18, 127, 18);

            // Outer circle and center dot
            u8g2.drawCircle(cx, cy, radius, U8G2_DRAW_ALL);
            u8g2.drawDisc(cx, cy, 2, U8G2_DRAW_ALL);

            // Draw 8 direction ticks and labels
            const char *labels[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
            u8g2.setFont(P_FONT);
            int labelRadius = radius + 12;
            for (int i = 0; i < 8; ++i)
            {
                float angDeg = i * 45.0f; // 0,45,90...
                float angRad = radians(angDeg);
                // tick endpoints (from inner to outer)
                int tx1 = cx + (int)((radius - tickOut) * sin(angRad));
                int ty1 = cy - (int)((radius - tickOut) * cos(angRad));
        int tx2 = cx + (int)((radius) * sin(angRad));
        int ty2 = cy - (int)((radius) * cos(angRad));
        u8g2.drawLine(tx1, ty1, tx2, ty2);

        // label position
        int lx = cx + (int)(labelRadius * sin(angRad));
        int ly = cy - (int)(labelRadius * cos(angRad));
        // Slight manual shifts for better centering
        if (i == 0)
            u8g2.drawStr(lx - 3, ly + 4, labels[i]); // N
        else if (i == 2)
            u8g2.drawStr(lx - 3, ly + 4, labels[i]); // E
        else if (i == 4)
            u8g2.drawStr(lx - 3, ly + 4, labels[i]); // S
        else if (i == 6)
            u8g2.drawStr(lx - 6, ly + 4, labels[i]); // W (shift more)
        else
            u8g2.drawStr(lx - 6, ly + 4, labels[i]); // diagonals
            }

            // Draw arrow pointing to current heading (0° = North up)
            float angleRad = radians(localHeadingDeg);
            int ax = cx + (int)(arrowLen * sin(angleRad));
            int ay = cy - (int)(arrowLen * cos(angleRad));
            u8g2.drawLine(cx, cy, ax, ay);

            // Arrow head (simple lines)
            float headOffset = 0.14f; // radians ~8°
            int leftx = cx + (int)((arrowLen - 5) * sin(angleRad + headOffset));
            int lefty = cy - (int)((arrowLen - 5) * cos(angleRad + headOffset));
            int rightx = cx + (int)((arrowLen - 5) * sin(angleRad - headOffset));
            int righty = cy - (int)((arrowLen - 5) * cos(angleRad - headOffset));
            u8g2.drawLine(ax, ay, leftx, lefty);
            u8g2.drawLine(ax, ay, rightx, righty);

            // Numeric heading display (degrees) at the bottom of the screen
            u8g2.setFont(P_FONT);
            int dispDeg = (int)round(localHeadingDeg) % 360;
            if (dispDeg < 0)
                dispDeg += 360;
            char degBuf[10];
            snprintf(degBuf, sizeof(degBuf), "%d%c", dispDeg, 176); // degree symbol

            int textx = cx - (u8g2.getStrWidth(degBuf) / 2);
            int texty = 127 - 2; // baseline near the bottom row (leave small padding)
            if (texty > 127)
                texty = 127;
            if (textx < 0)
                textx = 0;
            if (textx + u8g2.getStrWidth(degBuf) > 127)
                textx = 127 - u8g2.getStrWidth(degBuf);

            u8g2.drawStr(textx, texty, degBuf);
        }

    } while (u8g2.nextPage());
}
// void renderCompass()
// {
//     u8g2.firstPage();
//     do
//     {
//         // Header
//         u8g2.setFont(H_FONT);
//         u8g2.drawStr(startp, 16, "NAVIGATION");
//         u8g2.drawLine(0, 18, 127, 18);

//         // --- Selection prompt ---
//         if (states[3][1] == 0)
//         {
//             // u8g2.setFont(P_FONT);
//             // u8g2.drawStr(startp, 36, "Select the mode.");

//             // Option 0: "Compass"
//             if (states[3][0] == 0)
//             {
//                 u8g2.drawBox(0, 30, 128, 14); // highlight box
//                 u8g2.setDrawColor(0);
//                 u8g2.drawStr(6, 40, "Compass");
//                 u8g2.setDrawColor(1);
//             }
//             else
//             {
//                 u8g2.drawStr(6, 40, "Compass");
//             }

//             // Option 1: "Back Navigation"
//             if (states[3][0] == 1)
//             {
//                 u8g2.drawBox(0, 46, 128, 14); // highlight box
//                 u8g2.setDrawColor(0);
//                 u8g2.drawStr(6, 56, "Back Navigation");
//                 u8g2.setDrawColor(1);
//             }
//             else
//             {
//                 u8g2.drawStr(6, 56, "Back Navigation");
//             }
//         }
//         // --- Reverse (back) navigation mode ---
//         else if (states[3][1] == 1)
//         {
//             float localBearing = 0.0f, localHeading = 0.0f, localDist = 0.0f;

//             if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(50)) == pdTRUE)
//             {
//                 localBearing = navBearing;
//                 localHeading = navHeading;
//                 localDist = navDistance;
//                 xSemaphoreGive(gpsMutex);
//             }

//             // Center of navigation arrow
//             int cx = 64;
//             int cy = 50;
//             int len = 20;

//             // Draw center point
//             u8g2.drawCircle(cx, cy, 2, 1);

//             // Calculate needle end point (bearing relative to heading)
//             float angleRad = radians(localBearing - localHeading + 360.0f);
//             int ax = cx + (int)(len * sin(angleRad));
//             int ay = cy - (int)(len * cos(angleRad));

//             // Draw needle
//             u8g2.drawLine(cx, cy, ax, ay);

//             // Distance info
//             u8g2.setFont(P_FONT);
//             u8g2.setCursor(5, 10 + 15); // just below header
//             u8g2.print("Dist: ");
//             u8g2.print(localDist, 1);
//             u8g2.print(" m");
//         }
//         // --- Compass mode (normal compass) ---
//         else if (states[3][1] == 2)
//         {
//             // Read the latest heading safely (degrees 0..360)
//             float localHeadingDeg = 0.0f;
//             if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE)
//             {
//                 localHeadingDeg = compassHeading;
//                 xSemaphoreGive(i2cMutex);
//             }
//             else
//             {
//                 localHeadingDeg = compassHeading; // fallback, small staleness acceptable
//             }

//             // Normalise to [0,360)
//             if (!isfinite(localHeadingDeg))
//                 localHeadingDeg = 0.0f;
//             while (localHeadingDeg < 0.0f)
//                 localHeadingDeg += 360.0f;
//             while (localHeadingDeg >= 360.0f)
//                 localHeadingDeg -= 360.0f;

//             // Draw compass face
//             const int cx = 64; // center x
//             const int cy = 40; // center y (a bit higher to fit header + degree text)
//             const int radius = 22;
//             const int tickOut = 4; // tick length
//             const int arrowLen = 18;

//             // Outer circle and center dot
//             u8g2.drawCircle(cx, cy, radius, U8G2_DRAW_ALL);
//             u8g2.drawDisc(cx, cy, 2, U8G2_DRAW_ALL);

//             // Draw 8 direction ticks and labels
//             const char *labels[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
//             u8g2.setFont(P_FONT);
//             int labelRadius = radius + 10;
//             for (int i = 0; i < 8; ++i)
//             {
//                 float angDeg = i * 45.0f; // 0,45,90...
//                 float angRad = radians(angDeg);
//                 // tick endpoints (from inner to outer)
//                 int tx1 = cx + (int)((radius - tickOut) * sin(angRad));
//                 int ty1 = cy - (int)((radius - tickOut) * cos(angRad));
//                 int tx2 = cx + (int)((radius)*sin(angRad));
//                 int ty2 = cy - (int)((radius)*cos(angRad));
//                 u8g2.drawLine(tx1, ty1, tx2, ty2);

//                 // label position
//                 int lx = cx + (int)(labelRadius * sin(angRad));
//                 int ly = cy - (int)(labelRadius * cos(angRad));
//                 // Slight manual shifts for better centering
//                 if (i == 0)
//                     u8g2.drawStr(lx - 3, ly + 4, labels[i]); // N
//                 else if (i == 2)
//                     u8g2.drawStr(lx - 3, ly + 4, labels[i]); // E
//                 else if (i == 4)
//                     u8g2.drawStr(lx - 3, ly + 4, labels[i]); // S
//                 else if (i == 6)
//                     u8g2.drawStr(lx - 6, ly + 4, labels[i]); // W (shift more)
//                 else
//                     u8g2.drawStr(lx - 6, ly + 4, labels[i]); // diagonals
//             }

//             // Draw arrow pointing to current heading (0° = North up)
//             float angleRad = radians(localHeadingDeg);
//             int ax = cx + (int)(arrowLen * sin(angleRad));
//             int ay = cy - (int)(arrowLen * cos(angleRad));
//             u8g2.drawLine(cx, cy, ax, ay);

//             // Arrow head (simple lines)
//             float headOffset = 0.14f; // radians ~8°
//             int leftx = cx + (int)((arrowLen - 5) * sin(angleRad + headOffset));
//             int lefty = cy - (int)((arrowLen - 5) * cos(angleRad + headOffset));
//             int rightx = cx + (int)((arrowLen - 5) * sin(angleRad - headOffset));
//             int righty = cy - (int)((arrowLen - 5) * cos(angleRad - headOffset));
//             u8g2.drawLine(ax, ay, leftx, lefty);
//             u8g2.drawLine(ax, ay, rightx, righty);

//             // Numeric heading display (degrees) below the compass
//             u8g2.setFont(P_FONT);
//             int dispDeg = (int)round(localHeadingDeg) % 360;
//             if (dispDeg < 0)
//                 dispDeg += 360;
//             char degBuf[10];
//             snprintf(degBuf, sizeof(degBuf), "%d%c", dispDeg, 176); // degree symbol
//             int textx = cx - (u8g2.getStrWidth(degBuf) / 2);
//             int texty = cy + radius + 12;
//             u8g2.drawStr(textx, texty, degBuf);
//         }
//     } while (u8g2.nextPage());
// }
// void renderCompass()
// {
//     u8g2.firstPage();
//     do
//     {
//         // Header
//         u8g2.setFont(H_FONT);
//         u8g2.drawStr(startp, 16, "BACK NAVIGATION");
//         u8g2.drawLine(0, 18, 127, 18);

//         if (states[3][1] == 0)
//         {
//             // --- Prompt to start reverse navigation ---
//             u8g2.setFont(P_FONT);
//             u8g2.drawStr(startp, 36, "Do you want to start");
//             u8g2.drawStr(startp, 48, "back navigation?");

//             // Option 0: "No" at y = 62
//             if (states[3][0] == 0)
//             {
//                 u8g2.drawBox(0, 54, 128, 12);
//                 u8g2.setDrawColor(0);
//                 u8g2.drawStr(6, 63, "No");
//                 u8g2.setDrawColor(1);
//             }
//             else
//             {
//                 u8g2.drawStr(6, 63, "No");
//             }

//             // Option 1: "Yes" at y = 56
//             if (states[3][0] == 1)
//             {
//                 u8g2.drawBox(0, 66, 128, 12);
//                 u8g2.setDrawColor(0);
//                 u8g2.drawStr(6, 75, "Yes");
//                 u8g2.setDrawColor(1);
//             }
//             else
//             {
//                 u8g2.drawStr(6, 75, "Yes");
//             }
//         }
//         else
//         {
//             // --- Reverse navigation is ON ---
//             float localBearing, localHeading, localDist;

//             if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(50)) == pdTRUE)
//             {
//                 localBearing = navBearing;
//                 localHeading = navHeading;
//                 localDist = navDistance;
//                 xSemaphoreGive(gpsMutex);
//             }

//             // Center of compass circle
//             int cx = 64;
//             int cy = 50;
//             int len = 20;

//             // Draw center point
//             u8g2.drawCircle(cx, cy, 2, 1);

//             // Calculate needle end point
//             float angleRad = radians(localBearing - localHeading + 360.0f);
//             int ax = cx + len * sin(angleRad);
//             int ay = cy - len * cos(angleRad);

//             // Draw needle
//             u8g2.drawLine(cx, cy, ax, ay);

//             // Distance info
//             u8g2.setFont(P_FONT);
//             u8g2.setCursor(5, 10 + 15); // just below header
//             u8g2.print("Dist: ");
//             u8g2.print(localDist, 1);
//             u8g2.print(" m");
//         }

//     } while (u8g2.nextPage());
// }

void renderSent()
{
    const int VISIBLE = 11;      // number of lines shown on screen at once
    static int firstVisible = 0; // preserved between calls

    u8g2.firstPage();
    do
    {
        u8g2.clearBuffer();
        u8g2.setFont(H_FONT);
        u8g2.drawStr(startp, 10, "SENT BOX");
        u8g2.drawLine(0, 11, 127, 11);

        u8g2.setFont(P_FONT);
        int total = (int)sentBoxBucketPtr->size();

        if (total == 0)
        {
            u8g2.drawStr(startp, 25, "No messages");
        }
        else
        {
            // clamp selected index (use states[4] for SentBox)
            if (states[4][0] < 0)
                states[4][0] = 0;
            if (states[4][0] >= total)
                states[4][0] = total - 1;

            // ensure firstVisible is valid for current selection
            if (states[4][0] < firstVisible)
            {
                firstVisible = states[4][0];
            }
            else if (states[4][0] >= firstVisible + VISIBLE)
            {
                firstVisible = states[4][0] - VISIBLE + 1;
            }

            // clamp firstVisible bounds
            if (firstVisible < 0)
                firstVisible = 0;
            if (firstVisible > total - VISIBLE)
            {
                firstVisible = max(0, total - VISIBLE);
            }

            // draw visible messages (max VISIBLE)
            for (int row = 0; row < VISIBLE; row++)
            {
                int idx = firstVisible + row;
                if (idx >= total)
                    break;
                int y = 25 + row * 10;

                if (idx == states[4][0])
                {
                    u8g2.drawBox(0, y - 8, 128, 10);
                    u8g2.setDrawColor(0);
                    u8g2.drawStr(startp, y, sentBoxBucketPtr->at(idx).c_str());
                    u8g2.setDrawColor(1);
                }
                else
                {
                    u8g2.drawStr(startp, y, sentBoxBucketPtr->at(idx).c_str());
                }
            }

            // Show detail-flag if OK pressed (same behavior you had)
            if (states[4][1])
            {
                // ensure selected index is valid
                int sel = states[4][0];
                int total = (int)sentBoxBucketPtr->size();
                if (total <= 0)
                {
                    // nothing to show (shouldn't happen because outer code checked total > 0)
                }
                else
                {
                    if (sel < 0)
                        sel = 0;
                    if (sel >= total)
                        sel = total - 1;

                    // clear the current buffer and draw only the selected message
                    u8g2.clearBuffer();
                    u8g2.setFont(P_FONT);
                    u8g2.drawStr(startp, 10, "SENT MESSAGE"); // header

                    std::string msg = sentBoxBucketPtr->at(sel);
                    const int maxCharsPerLine = 16; // tune to match your font/width
                    const int yStart = 25;
                    const int lineSpacing = 10;

                    // Draw message wrapped into fixed-width chunks
                    for (size_t pos = 0, line = 0; pos < msg.length(); pos += maxCharsPerLine, ++line)
                    {
                        std::string part = msg.substr(pos, maxCharsPerLine);
                        int y = yStart + line * lineSpacing;
                        if (y > 54)
                            break; // avoid overwriting footer area
                        u8g2.drawStr(startp, y, part.c_str());
                    }

                    // // footer: OK → Back
                    // u8g2.drawBox(0, 58, 128, 6);
                    // u8g2.setDrawColor(0);
                    // u8g2.drawStr(startp, 63, "Press OK"); // arrow char optional
                    // u8g2.setDrawColor(1);

                    // send this buffer for immediate display (keeps behavior similar to your SEND screen)
                    u8g2.sendBuffer();
                }
            }
        }
    } while (u8g2.nextPage());
}

void _OledDisplayTask(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
        {
            switch (page)
            {
            case 0:
                renderWelcome();
                break;
            case 1:
                renderInbox();
                break;
            case 2:
                renderSend();
                break;
            case 3:
                renderCompass();
                break;
            case 4:
                renderSent();
                break;
            }
            xSemaphoreGive(xSemaphore);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// LORA
void _LoRaListenTask(void *pvParameters)
{

    while (1)
    {
        int packetSize = LoRa.parsePacket();
        if (packetSize)
        {
            RSSI = LoRa.packetRssi();
            int snr = LoRa.packetSnr();
            Serial.printf("--------[LoRa Received] RSSI: %d, SNR: %d --------\n", RSSI, snr);
            std::vector<int8_t> newReceivedPayload;
            std::string jsonPayload;
            while (LoRa.available())
            {
                newReceivedPayload.push_back(LoRa.read());
            }

            bool valid = userDevice.receive(newReceivedPayload);
            printf("Payload Valid : %s\n", valid ? "[True]" : "[False]");
            // userDevice.setPayload(newReceivedPayload);
            if (valid)
            {
                jsonPayload = userDevice.getJsonPayload();
                Serial.printf("[Payload Received] : %s\n", jsonPayload.c_str());
                beep();
                Serial.println(jsonPayload.c_str());
                userDevice.printAckBucket();
            }
            else
            {
                userDevice.printAckBucket();
            }
            // handleReceivedPayload(receivedPayload);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Check for new LoRa messages at a regular interval
    }
}

void _GpsUpdateTask(void *parameter)
{
    while (1)
    { // 30 seconds pushing to gps bucket
        static uint32_t last = 0;
        if (millis() - last > 30000 && gpsFix)
        {
            // blinkBLUE();
            Serial.println("[*] 30 Min - Pushing to gps bucket");
            userDevice.setGpsNew(currentDeviceGPS.latitude, currentDeviceGPS.longitude);
            userDevice.createGps(currentDeviceGPS.latitude, currentDeviceGPS.longitude, 0);
            std::vector<int8_t> *payloadVector = new std::vector<int8_t>(userDevice.getPayload());
            xQueueSend(loraQueue, &payloadVector, portMAX_DELAY); // send selection

            Serial.println("[GPS String Bucket]");
            if (gpsStringBucketPtr)
            {
                for (const auto &gpsString : *gpsStringBucketPtr)
                {
                    Serial.println(gpsString.c_str());
                }
            }
            last = millis();
        }
        while (GPS_Serial.available() > 0)
        {
            char c = GPS_Serial.read();
            gps.encode(c);

            if (gps.location.isUpdated())
            {
                xSemaphoreTake(gpsMutex, portMAX_DELAY);
                currentDeviceGPS.latitude = gps.location.lat();
                currentDeviceGPS.longitude = gps.location.lng();
                gpsFix = gps.location.isValid();
                xSemaphoreGive(gpsMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void _LoRaSendTask(void *pvParameters)
{
    for (;;)
    {
        std::vector<int8_t> *receivedVec;
        if (xQueueReceive(loraQueue, &receivedVec, portMAX_DELAY))
        {
            bool pass = userDevice.setPayload(*receivedVec);
            userDevice.loraSend();
            if ((*receivedVec)[8] != 2)
            {
                userDevice.setSentboxNew(userDevice.getMsg(*receivedVec));
            }
            // userDevice.printPayload();
            delete receivedVec; // free when done

            if (states[2][1] == 1)
            {
                vTaskDelay(400 / portTICK_PERIOD_MS); // simulate sending delay
                states[2][1] = 0;                     // reset sending flag
                beep2();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed
    }
}

void _BackNavigationTask(void *pvParameters)
{
    // snapshot variables are kept static so they persist across iterations
    static bool reverseSnapshotTaken = false;
    static std::vector<UserDevicePayload::Coordinate> reverseBucket;

    for (;;)
    {
        if ((states[3][1] == 1 || states[3][1] == 2) && page == 3)
        {
            // read sensors
            int16_t magX, magY, magZ;
            int16_t accX, accY, accZ;
            readMagnetometer(magX, magY, magZ);
            readAccelerometer(accX, accY, accZ);

            // tilt compensation (unchanged)
            float axn = (float)accX;
            float ayn = (float)accY;
            float azn = (float)accZ;
            float norm = sqrt(axn * axn + ayn * ayn + azn * azn);
            if (norm == 0.0f)
                norm = 1.0f;
            axn /= norm;
            ayn /= norm;
            azn /= norm;

            float xh = (float)magX * azn - (float)magZ * axn;
            float yh = (float)magY * azn - (float)magZ * ayn;
            float newHeading = atan2(yh, xh) * 180.0f / PI;
            if (newHeading < 0)
            {
                newHeading += 360.0f;
            }

            if (states[3][1] == 1)
            {

                Serial.println("[BackNavigation] Active");
                // If we haven't taken the snapshot yet, do it now (copy gpsBucket)
                if (!reverseSnapshotTaken)
                {
                    // copy the current gpsBucket into reverseBucket under gpsMutex
                    if (xSemaphoreTake(gpsMutex, portMAX_DELAY) == pdTRUE)
                    {
                        // safe copy of the entire trail
                        reverseBucket = *gpsBucket; // copy constructor
                        // initialize targetIndex similarly to your previous logic
                        int localPathIndex = (int)reverseBucket.size() - 1;
                        if (targetIndex < 0)
                        {
                            targetIndex = (localPathIndex >= 1) ? (localPathIndex - 1) : -1;
                        }
                        xSemaphoreGive(gpsMutex);
                        reverseSnapshotTaken = true;
                        Serial.println("[BackNavigation] Snapshot taken");
                    }
                    else
                    {
                        // Couldn't take gpsMutex - try again next loop
                        vTaskDelay(pdMS_TO_TICKS(200));
                        continue;
                    }
                }

                // update heading globals (unchanged)
                heading = newHeading;
                navHeading = newHeading;

                // pathIndex now comes from the snapshot if present, otherwise fall back (shouldn't happen during reverse nav)
                int pathIndexLocal = -1;
                if (reverseSnapshotTaken)
                {
                    pathIndexLocal = (int)reverseBucket.size() - 1;
                }
                else
                {
                    // fallback to live gpsBucket (kept for safety; original behavior)
                    pathIndexLocal = (int)gpsBucket->size() - 1;
                }

                if (pathIndexLocal >= 0)
                {
                    // initialize targetIndex if not set (we already tried to set it during snapshot, but keep the check)
                    if (targetIndex < 0)
                    {
                        if (xSemaphoreTake(gpsMutex, portMAX_DELAY) == pdTRUE)
                        {
                            targetIndex = (pathIndexLocal >= 1) ? (pathIndexLocal - 1) : -1;
                            xSemaphoreGive(gpsMutex);
                        }
                    }

                    // copy target waypoint safely from the correct bucket (snapshot preferred)
                    UserDevicePayload::Coordinate tgt;
                    if (xSemaphoreTake(gpsMutex, portMAX_DELAY) == pdTRUE)
                    {
                        bool valid = false;
                        if (reverseSnapshotTaken)
                        {
                            if (targetIndex >= 0 && targetIndex < (int)reverseBucket.size())
                            {
                                tgt = reverseBucket[targetIndex];
                                valid = true;
                            }
                        }
                        else
                        {
                            if (targetIndex >= 0 && targetIndex < pathIndexLocal)
                            {
                                tgt = (*gpsBucket)[targetIndex];
                                valid = true;
                            }
                        }

                        if (!valid)
                        {
                            // no valid target -> release mutex and wait briefly
                            xSemaphoreGive(gpsMutex);
                            vTaskDelay(pdMS_TO_TICKS(200));
                            continue;
                        }

                        xSemaphoreGive(gpsMutex);
                    }
                    else
                    {
                        vTaskDelay(pdMS_TO_TICKS(200));
                        continue;
                    }

                    // compute distance & bearing (unchanged)
                    float dist = distanceBetween(currentDeviceGPS.latitude, currentDeviceGPS.longitude, tgt.latitude, tgt.longitude);
                    float bearing = calculateBearing(currentDeviceGPS.latitude, currentDeviceGPS.longitude, tgt.latitude, tgt.longitude);

                    // read current heading under i2cMutex (avoid racing with CompassTask)
                    float currHeadingLocal = heading;
                    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                    {
                        currHeadingLocal = heading;
                        xSemaphoreGive(i2cMutex);
                    }

                    // update globals (protected by gpsMutex)
                    if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                    {
                        navDistance = dist;
                        navBearing = bearing;
                        navHeading = currHeadingLocal;

                        // if arrived at this waypoint, decrement targetIndex
                        if (dist <= ARRIVE_DIST_METERS && targetIndex > 0)
                        {
                            targetIndex--;
                        }

                        xSemaphoreGive(gpsMutex);
                    }
                }
            }
            else if (states[3][1] == 2 && page == 3)
            {
                Serial.println("[Compass Mode] Active");
                float newHeading = atan2(yh, xh) * 180.0f / PI;
                if (newHeading < 0.0f)
                    newHeading += 360.0f;

                // ---- update shared global under mutex to avoid races with render/other tasks ----
                if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    heading = newHeading;        // keep existing global consistent
                    compassHeading = newHeading; // this is what renderCompass() should read
                    xSemaphoreGive(i2cMutex);
                }
                else
                {
                    // If we can't grab the mutex, still update the volatile so UI gets approximate value
                    heading = newHeading;
                    compassHeading = newHeading;
                }
            }
            else
            {
                // If reverse nav is not active or we left page 3, clear the snapshot
                // so the next time reverse nav is entered we take a fresh copy.
                // This keeps the snapshot lifetime bound to the reverse-nav session.
                static bool wasSnapshotCleared = false;
                if (!wasSnapshotCleared)
                {
                    // Clear snapshot and reset related indices
                    // (No mutex needed to clear reverseBucket since only this task touches reverseBucket)
                    reverseBucket.clear();
                    reverseSnapshotTaken = false;
                    targetIndex = -1; // reset so we'll re-initialize when entering reverse nav again
                    wasSnapshotCleared = true;
                    Serial.println("[BackNavigation] Snapshot cleared");
                }
                // reset the flag so next time through when reverse nav becomes active we will take snapshot
                if (states[3][1] == 1 && page == 3)
                    wasSnapshotCleared = false;
            }
        }
        // Compass mode - just

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}
// void _BackNavigationTask(void *pvParameters)
// {
//     for (;;)
//     {
//         if (states[3][1] == 1 && page == 3)
//         {
//             Serial.println("[BackNavigation] Active");
//             // read sensors
//             int16_t magX, magY, magZ;
//             int16_t accX, accY, accZ;
//             readMagnetometer(magX, magY, magZ);
//             readAccelerometer(accX, accY, accZ);

//             // tilt compensation
//             float axn = (float)accX;
//             float ayn = (float)accY;
//             float azn = (float)accZ;
//             float norm = sqrt(axn * axn + ayn * ayn + azn * azn);
//             if (norm == 0.0f)
//                 norm = 1.0f;
//             axn /= norm;
//             ayn /= norm;
//             azn /= norm;

//             float xh = (float)magX * azn - (float)magZ * axn;
//             float yh = (float)magY * azn - (float)magZ * ayn;
//             float newHeading = atan2(yh, xh) * 180.0f / PI;
//             if (newHeading < 0)
//                 newHeading += 360.0f;

//             heading = newHeading;              // update global heading
//             navHeading = newHeading;           // also mirror to navHeading for reverse nav usage
//             pathIndex = gpsBucket->size() - 1; // update pathIndex from gpsBucket size

//             if (pathIndex >= 0)
//             {
//                 // initialize targetIndex if not set
//                 if (targetIndex < 0)
//                 {
//                     if (xSemaphoreTake(gpsMutex, portMAX_DELAY) == pdTRUE)
//                     {
//                         targetIndex = pathIndex - 1;
//                         xSemaphoreGive(gpsMutex);
//                     }
//                 }
//                 // current GPS read (no gpsMutex required to read TinyGPS directly)

//                 // copy target waypoint safely
//                 UserDevicePayload::Coordinate tgt;
//                 if (xSemaphoreTake(gpsMutex, portMAX_DELAY) == pdTRUE)
//                 {
//                     if (targetIndex >= 0 && targetIndex < pathIndex)
//                     {
//                         tgt = (*gpsBucket)[targetIndex];
//                     }
//                     else
//                     {
//                         // no valid target
//                         xSemaphoreGive(gpsMutex);
//                         vTaskDelay(pdMS_TO_TICKS(200));
//                         continue;
//                     }
//                     xSemaphoreGive(gpsMutex);
//                 }
//                 else
//                 {
//                     vTaskDelay(pdMS_TO_TICKS(200));
//                     continue;
//                 }

//                 // compute distance & bearing
//                 float dist = distanceBetween(currentDeviceGPS.latitude, currentDeviceGPS.longitude, tgt.latitude, tgt.longitude);
//                 float bearing = calculateBearing(currentDeviceGPS.latitude, currentDeviceGPS.longitude, tgt.latitude, tgt.longitude);

//                 // read current heading under i2cMutex (avoid racing with CompassTask)
//                 float currHeadingLocal = heading;
//                 if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE)
//                 {
//                     currHeadingLocal = heading;
//                     xSemaphoreGive(i2cMutex);
//                 }

//                 // update globals (protected by gpsMutex)
//                 if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(50)) == pdTRUE)
//                 {
//                     navDistance = dist;
//                     navBearing = bearing;
//                     navHeading = currHeadingLocal;
//                     // if arrived at this waypoint, decrement targetIndex
//                     if (dist <= ARRIVE_DIST_METERS && targetIndex > 0)
//                     {
//                         targetIndex--;
//                     }
//                     xSemaphoreGive(gpsMutex);
//                 }
//             }
//         }
//         vTaskDelay(pdMS_TO_TICKS(300));
//     }
// }

// void _CompassTask(void *pvParameters)
// {
//     for (;;)
//     {
//         // Run only when Compass mode is active and compass page is visible
//         if (states[3][1] == 2 && page == 3)
//         {
//             // read sensors (use the same sensor-read helpers you already have)
//             int16_t magX, magY, magZ;
//             int16_t accX, accY, accZ;
//             readMagnetometer(magX, magY, magZ);
//             readAccelerometer(accX, accY, accZ);

//             // ---- tilt compensation ----
//             float axn = (float)accX;
//             float ayn = (float)accY;
//             float azn = (float)accZ;

//             float norm = sqrt(axn * axn + ayn * ayn + azn * azn);
//             if (norm == 0.0f)
//                 norm = 1.0f;
//             axn /= norm;
//             ayn /= norm;
//             azn /= norm;

//             // compute horizontal magnetometer components (tilt-compensated)
//             float xh = (float)magX * azn - (float)magZ * axn;
//             float yh = (float)magY * azn - (float)magZ * ayn;

//             // heading in degrees (0..360, 0 = North)
//             float newHeading = atan2(yh, xh) * 180.0f / PI;
//             if (newHeading < 0.0f)
//                 newHeading += 360.0f;

//             // ---- update shared global under mutex to avoid races with render/other tasks ----
//             if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE)
//             {
//                 heading = newHeading;        // keep existing global consistent
//                 compassHeading = newHeading; // this is what renderCompass() should read
//                 xSemaphoreGive(i2cMutex);
//             }
//             else
//             {
//                 // If we can't grab the mutex, still update the volatile so UI gets approximate value
//                 heading = newHeading;
//                 compassHeading = newHeading;
//             }
//         }

//         // match update rate of reverse nav (about 3 Hz)
//         vTaskDelay(pdMS_TO_TICKS(300));
//     }
// }
// -------------------- Sensor helpers --------------------

void initAccelerometerAndMagnetometer()
{
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        // QMC5883L soft reset
        Wire.beginTransmission(QMC5883L_ADDR);
        Wire.write(0x0B);
        Wire.write(0x01);
        Wire.endTransmission();
        delay(50);
        // continuous mode / register 0x09 configuration (as used previously)
        Wire.beginTransmission(QMC5883L_ADDR);
        Wire.write(0x09);
        Wire.write(0x1D);
        Wire.endTransmission();

        // Wake MPU6500
        Wire.beginTransmission(MPU6500_ADDR);
        Wire.write(0x6B);
        Wire.write(0x00);
        Wire.endTransmission();

        xSemaphoreGive(i2cMutex);
    }
    else
    {
        Serial.println("initSensors: couldn't take i2cMutex");
    }
}

// read 6 bytes from QMC5883L starting register 0x00 (little-endian)
void readMagnetometer(int16_t &mx_out, int16_t &my_out, int16_t &mz_out)
{
    Wire.beginTransmission(QMC5883L_ADDR);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.requestFrom((uint8_t)QMC5883L_ADDR, (uint8_t)6); // Cast to uint8_t
    if (Wire.available() >= 6)
    {
        uint8_t lx = Wire.read();
        uint8_t hx = Wire.read();
        uint8_t ly = Wire.read();
        uint8_t hy = Wire.read();
        uint8_t lz = Wire.read();
        uint8_t hz = Wire.read();
        mx_out = (int16_t)((uint16_t)hx << 8 | (uint16_t)lx);
        my_out = (int16_t)((uint16_t)hy << 8 | (uint16_t)ly);
        mz_out = (int16_t)((uint16_t)hz << 8 | (uint16_t)lz);
    }
    else
    {
        // keep previous or set zero if needed
    }
}

void readAccelerometer(int16_t &ax_out, int16_t &ay_out, int16_t &az_out)
{
    Wire.beginTransmission(MPU6500_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission();

    Wire.requestFrom((uint8_t)MPU6500_ADDR, (uint8_t)6); // Cast to uint8_t
    if (Wire.available() >= 6)
    {
        uint8_t axh = Wire.read();
        uint8_t axl = Wire.read();
        uint8_t ayh = Wire.read();
        uint8_t ayl = Wire.read();
        uint8_t azh = Wire.read();
        uint8_t azl = Wire.read();
        ax_out = (int16_t)((uint16_t)axh << 8 | (uint16_t)axl);
        ay_out = (int16_t)((uint16_t)ayh << 8 | (uint16_t)ayl);
        az_out = (int16_t)((uint16_t)azh << 8 | (uint16_t)azl);
    }
    else
    {
        // keep previous
    }
}

// haversine distance (meters)
float distanceBetween(float lat1, float lon1, float lat2, float lon2)
{
    const float R = 6371000.0f;
    float dLat = radians(lat2 - lat1);
    float dLon = radians(lon2 - lon1);
    lat1 = radians(lat1);
    lat2 = radians(lat2);
    float a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

// initial bearing from point1 to point2 in degrees (0..360)
float calculateBearing(float lat1, float lon1, float lat2, float lon2)
{
    float dLon = radians(lon2 - lon1);
    lat1 = radians(lat1);
    lat2 = radians(lat2);
    float y = sin(dLon) * cos(lat2);
    float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    float brng = atan2(y, x);
    brng = degrees(brng);
    return fmod(brng + 360.0f, 360.0f);
}

void setupBuzzer()
{
    // Initialize the buzzer pin as output
    pinMode(BUZZER_PIN, OUTPUT);

    // For passive buzzer, set up PWM
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(BUZZER_PIN, PWM_CHANNEL);

    // Ensure buzzer is off initially
    noTone();
}

// Function for a simple short beep (works for active or passive buzzer)
void beep()
{
// For active buzzer: turn on for 100ms
// For passive buzzer: generate a 1000Hz tone for 100ms
#ifdef ACTIVE_BUZZER
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
#else
    ledcWriteTone(PWM_CHANNEL, 1000); // 1000Hz tone
    delay(100);
    noTone();
#endif
}

// Function for notification sound (two short beeps)
void beep2()
{
    beep();
    delay(100); // Short pause between beeps
    beep();
}

// Function for error sound (longer, lower-pitched beeps)
void beep3()
{
#ifdef ACTIVE_BUZZER
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500); // Longer beep
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
#else
    ledcWriteTone(PWM_CHANNEL, 500); // Lower 500Hz tone for error
    delay(500);
    noTone();
    delay(100);
    ledcWriteTone(PWM_CHANNEL, 500);
    delay(500);
    noTone();
#endif
}

// Helper function to stop tone (for passive buzzer)
void noTone()
{
    ledcWrite(PWM_CHANNEL, 0); // Stop PWM signal
}

float readBatteryVoltage()
{
    long sum = 0;
    const int samples = 20; // averaging for stability
    for (int i = 0; i < samples; i++)
    {
        sum += analogRead(BATTERY_PIN);
        delay(5);
    }
    int rawADC = sum / samples;

    float adcVoltage = (rawADC / 4095.0) * 3.3; // pin voltage
    float batteryVoltage = adcVoltage * ((R1 + R2) / R2);

    return batteryVoltage;
}

void handleMessageFromBLE(const std::string &msg)
{
    Serial.printf("Handling message from BLE (Main.cpp) : %s\n", msg.c_str());
    userDevice.createCmsg(msg, 0);
    std::vector<int8_t> *payloadVector = new std::vector<int8_t>(userDevice.getPayload());
    xQueueSend(loraQueue, &payloadVector, portMAX_DELAY); // send selection
}