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

UserDevicePayload userDevice(77);
std::vector<std::string> PredefinedMessages = userDevice.getPredefinedMessagesForUser();
UserDevicePayload::Coordinate currentDeviceGPS;
bool gpsFix = false;
std::vector<std::string> *inboxBucketPtr;
std::vector<std::string> *sentBoxBucketPtr;
std::vector<std::string> *gpsStringBucketPtr;
std::vector<UserDevicePayload::Coordinate> *gpsBucket;
// BLE
BLEVectorSyncServer bleServer("LORA10_Device");
BLEVectorSync *gpsSync;
BLEVectorSync *inboxSync;
BLEVectorSync *sendboxSync;

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
volatile float heading = 0.0f;     // updated by CompassTask (deg)
volatile float navDistance = 0.0f; // distance to current target (m)
volatile float navBearing = 0.0f;  // bearing to current target (deg)
volatile float navHeading = 0.0f;  // current compass heading (deg)

// OLED
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
U8G2_SSD1309_128X128_NONAME0_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);  // fore Big one
#define H_FONT u8g2_font_ncenB08_tr
#define P_FONT u8g2_font_6x10_tr

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
    xTaskCreatePinnedToCore(_LoRaListenTask, "LoRaListenTask", 8192, NULL, 1, &Task5, 0); // xTaskCreatePinnedToCore(_BleCommunicationTask, "BleCommunicationTask", 2048, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(_LoRaSendTask, "LoRaSendTask", 2048, NULL, 1, &Task1, 0);     // xTaskCreatePinnedToCore(_BleCommunicationTask, "BleCommunicationTask", 2048, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(_GpsUpdateTask, "GpsUpdateTask", 4096, NULL, 1, &Task4, 1);

    xTaskCreatePinnedToCore(_ButtonPressTask, "ButtonPressTask", 2048, NULL, 2, &Task2, 1);
    xTaskCreatePinnedToCore(_OledDisplayTask, "OledDisplayTask", 4096, NULL, 1, &Task3, 1);
    // xTaskCreatePinnedToCore(_SleepManagerTask, "SleepManagerTask", 4096, NULL, 2, &SleepManagerTaskHandle, 1);
    xTaskCreatePinnedToCore(_BackNavigationTask, "BackNavigationTask", 4096, NULL, 2, &Task6, 0);

    // digitalWrite(BLUE_LED, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
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
        if (page == 3 && states[3][1] == 1) // Compass
        {
            states[3][1] = 0; // Toggle compass off
        }
        page = (page + 1) % 5; // Cycle through screens 0 to 4
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
            Serial.println(userDevice.getJsonPayload().c_str());
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
        if (page == 1 && states[1][0] >= 0 && states[1][0] <= 1) // Inbox
        {
            states[1][0] = (states[1][0] + 1) % 2;
        }
        else if (page == 2) // Send
        {
            states[2][1] = 1;
            userDevice.createPmsg(states[2][0], 0); // Create predefined message
            std::vector<int8_t> *payloadVector = new std::vector<int8_t>(userDevice.getPayload());
            // flag as "sending"
            // int selectedMsg = states[2][0]; // selection index
            xQueueSend(loraQueue, &payloadVector, portMAX_DELAY); // send selection
            // set to 0 after sending
        }
        else if (page == 3) // Compass
        {
            if (states[3][0] == 1)
            {
                states[3][1] = 1; // Set detail flag
            }
            else
            {
                states[3][1] = 0; // Reset detail flag
            }
        }
        else if (page == 4 && states[4][0] >= 0 && states[4][0] <= 1) // SentBox
        {
            states[4][0] = (states[4][0] + 1) % 2;
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
                        // beep2();
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
    u8g2.firstPage();
    do
    {
        u8g2.clearBuffer();
        u8g2.setFont(H_FONT);           // Set header font
        u8g2.drawStr(startp, 10, "HOME"); // Display welcome message
        u8g2.drawLine(0, 11, 127, 11);  // Draw horizontal line below the text

        u8g2.setFont(H_FONT);
        char rssiBuf[16];
        sprintf(rssiBuf, "RSSI : %d", RSSI);
        int rssiWidth = u8g2.getStrWidth(rssiBuf);
        u8g2.drawStr((128 - rssiWidth) / 2, 30, rssiBuf); // centered

        // --- GPS Status ---
        const char* gpsStr = gpsFix ? "GPS : Active" : "GPS : Waiting...";
        int gpsWidth = u8g2.getStrWidth(gpsStr);
        u8g2.drawStr((128 - gpsWidth) / 2, 45, gpsStr);
    } while (u8g2.nextPage());
}

void renderInbox()
{
    u8g2.firstPage();
    do
    {
        u8g2.clearBuffer();
        u8g2.setFont(H_FONT);
        u8g2.drawStr(startp, 10, "INBOX");
        u8g2.drawLine(0, 11, 127, 11);

        u8g2.setFont(P_FONT);
        if (inboxBucketPtr->size() == 0)
        {
            u8g2.drawStr(startp, 25, "No messages");
        }
        else
        {
            for (int i = 0; i < inboxBucketPtr->size() && i < 5; i++)
            {
                int y = 25 + i * 10;
                if (i == states[1][0])
                {
                    u8g2.drawBox(0, y - 8, 128, 10);
                    u8g2.setDrawColor(0);
                    u8g2.drawStr(2, y, inboxBucketPtr->at(i).c_str());
                    u8g2.setDrawColor(1);
                }
                else
                {
                    u8g2.drawStr(2, y, inboxBucketPtr->at(i).c_str());
                }
            }
            // Show detail-flag if OK pressedxxx
            if (states[1][1])
            {
                u8g2.drawBox(0, 58, 128, 6);
                u8g2.setDrawColor(0);
                u8g2.drawStr(startp, 63, "OK → Open Msg");
                u8g2.setDrawColor(1);
            }
        }
    } while (u8g2.nextPage());
}

void renderSend()
{
    const int VISIBLE = 4;       // number of lines shown on screen at once
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
        // Header
        u8g2.setFont(H_FONT);
        u8g2.drawStr(startp, 16, "BACK NAVIGATION");
        u8g2.drawLine(0, 18, 127, 18);

        if (states[3][1] == 0)
        {
            // --- Prompt to start reverse navigation ---
            u8g2.setFont(P_FONT);
            u8g2.drawStr(startp, 36, "Do you want to start");
            u8g2.drawStr(startp, 48, "back navigation?");

            // Option 0: "No" at y = 62
            if (states[3][0] == 0)
            {
                u8g2.drawBox(0, 38, 128, 10);
                u8g2.setDrawColor(0);
                u8g2.drawStr(6, 46, "No");
                u8g2.setDrawColor(1);
            }
            else
            {
                u8g2.drawStr(6, 46, "No");
            }

            // Option 1: "Yes" at y = 56
            if (states[3][0] == 1)
            {
                u8g2.drawBox(0, 48, 128, 10);
                u8g2.setDrawColor(0);
                u8g2.drawStr(6, 56, "Yes");
                u8g2.setDrawColor(1);
            }
            else
            {
                u8g2.drawStr(6, 56, "Yes");
            }
        }
        else
        {
            // --- Reverse navigation is ON ---
            float localBearing, localHeading, localDist;

            if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                localBearing = navBearing;
                localHeading = navHeading;
                localDist = navDistance;
                xSemaphoreGive(gpsMutex);
            }

            // Center of compass circle
            int cx = 64;
            int cy = 50;
            int len = 20;

            // Draw center point
            u8g2.drawCircle(cx, cy, 2, 1);

            // Calculate needle end point
            float angleRad = radians(localBearing - localHeading + 360.0f);
            int ax = cx + len * sin(angleRad);
            int ay = cy - len * cos(angleRad);

            // Draw needle
            u8g2.drawLine(cx, cy, ax, ay);

            // Distance info
            u8g2.setFont(P_FONT);
            u8g2.setCursor(5, 10 + 15); // just below header
            u8g2.print("Dist: ");
            u8g2.print(localDist, 1);
            u8g2.print(" m");
        }

    } while (u8g2.nextPage());
}


void renderSent()
{
    u8g2.firstPage();
    do
    {
        u8g2.clearBuffer();
        u8g2.setFont(H_FONT);
        u8g2.drawStr(startp, 10, "SENT BOX");
        u8g2.drawLine(0, 11, 127, 11);

        u8g2.setFont(P_FONT);
        if (sentBoxBucketPtr->size() == 0)
        {
            u8g2.drawStr(startp, 25, "No messages");
        }
        else
        {
            for (int i = 0; i < sentBoxBucketPtr->size() && i < 5; i++)
            {
                int y = 25 + i * 10;
                if (i == states[1][0])
                {
                    u8g2.drawBox(0, y - 8, 128, 10);
                    u8g2.setDrawColor(0);
                    u8g2.drawStr(startp, y, sentBoxBucketPtr->at(i).c_str());
                    u8g2.setDrawColor(1);
                }
                else
                {
                    u8g2.drawStr(startp, y, sentBoxBucketPtr->at(i).c_str());
                }
            }
            // Show detail-flag if OK pressedxxx
            if (states[1][1])
            {
                u8g2.drawBox(0, 58, 128, 6);
                u8g2.setDrawColor(0);
                u8g2.drawStr(startp, 63, "OK → Open Msg");
                u8g2.setDrawColor(1);
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
            Serial.println("---------------Packet Received]");
            Serial.println("[---------------Received packet]");
            std::vector<int8_t> newReceivedPayload;
            std::string jsonPayload;
            RSSI = LoRa.packetRssi();
            while (LoRa.available())
            {
                newReceivedPayload.push_back(LoRa.read());
            }

            userDevice.receive(newReceivedPayload);
            // userDevice.setPayload(newReceivedPayload);
            if (1)
            {
                jsonPayload = userDevice.getJsonPayload();
                Serial.println("[Payload Received]");
                Serial.println(jsonPayload.c_str());
                // baseDevice.printAckBucket();
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
            states[2][1] = 0;   // reset sending flag
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed
    }
}

void _BackNavigationTask(void *pvParameters)
{
    for (;;)
    {
        if (states[3][1] == 1 && page == 3)
        {
            Serial.println("[BackNavigation] Active");
            // read sensors
            int16_t magX, magY, magZ;
            int16_t accX, accY, accZ;
            readMagnetometer(magX, magY, magZ);
            readAccelerometer(accX, accY, accZ);

            // tilt compensation
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
                newHeading += 360.0f;

            heading = newHeading;    // update global heading
            navHeading = newHeading; // also mirror to navHeading for reverse nav usage
            pathIndex = gpsBucket->size() - 1 ; // update pathIndex from gpsBucket size  

            if (pathIndex >= 0)
            {
                // initialize targetIndex if not set
                if (targetIndex < 0)
                {
                    if (xSemaphoreTake(gpsMutex, portMAX_DELAY) == pdTRUE)
                    {
                        targetIndex = pathIndex - 1;
                        xSemaphoreGive(gpsMutex);
                    }
                }
                // current GPS read (no gpsMutex required to read TinyGPS directly)

                // copy target waypoint safely
                UserDevicePayload::Coordinate tgt;
                if (xSemaphoreTake(gpsMutex, portMAX_DELAY) == pdTRUE)
                {
                    if (targetIndex >= 0 && targetIndex < pathIndex)
                    {
                        tgt = (*gpsBucket)[targetIndex];
                    }
                    else
                    {
                        // no valid target
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

                // compute distance & bearing
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
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}
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


void setupBuzzer() {
  // Initialize the buzzer pin as output
  pinMode(BUZZER_PIN, OUTPUT);
  
  // For passive buzzer, set up PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, PWM_CHANNEL);
  
  // Ensure buzzer is off initially
  noTone();
}

// Function for a simple short beep (works for active or passive buzzer)
void beep() {
  // For active buzzer: turn on for 100ms
  // For passive buzzer: generate a 1000Hz tone for 100ms
  #ifdef ACTIVE_BUZZER
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
  #else
    ledcWriteTone(PWM_CHANNEL, 1000);  // 1000Hz tone
    delay(100);
    noTone();
  #endif
}

// Function for notification sound (two short beeps)
void beep2() {
  beep();
  delay(100);  // Short pause between beeps
  beep();
}

// Function for error sound (longer, lower-pitched beeps)
void beep3() {
  #ifdef ACTIVE_BUZZER
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);  // Longer beep
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
  #else
    ledcWriteTone(PWM_CHANNEL, 500);  // Lower 500Hz tone for error
    delay(500);
    noTone();
    delay(100);
    ledcWriteTone(PWM_CHANNEL, 500);
    delay(500);
    noTone();
  #endif
}

// Helper function to stop tone (for passive buzzer)
void noTone() {
  ledcWrite(PWM_CHANNEL, 0);  // Stop PWM signal
}
