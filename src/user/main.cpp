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

void _LoRaListenTask(void *pvParameters);
void _ButtonPressTask(void *pvParameters);
void _OledDisplayTask(void *pvParameters);
void _GpsUpdateTask(void *pvParameters);
void _LoRaSendTask(void *pvParameters);
void _SleepManagerTask(void *pvParameters);

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
    {OK_BTN, "OK", 50, okBtnPressed, false, 0} // long debounce
};
const size_t BUTTON_COUNT = sizeof(buttons) / sizeof(buttons[0]);
int page = 0;
int states[5][2] = {0};
SemaphoreHandle_t xSemaphore;
QueueHandle_t loraQueue;
//----------------------

// #define RED_LED 35
// #define BLUE_LED 36
// void blinkRED();
// void blinkBLUE();

// OLED
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#define H_FONT u8g2_font_ncenB08_tr
#define P_FONT u8g2_font_6x10_tr

// GPS
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
HardwareSerial GPS_Serial(2);
TinyGPSPlus gps;
SemaphoreHandle_t gpsMutex;

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
    Serial.println("[+] Button handlers initialized.");

    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if (!LoRa.begin(433E6))
    {
        Serial.println("LoRa init failed.");
        while (1)
            ;
    }
    // LoRa.setSyncWord(0x12);
    // LoRa.setTxPower(15);
    // LoRa.setSpreadingFactor(13);
    Serial.println("[+] LoRa init succeeded.");
    xSemaphore = xSemaphoreCreateMutex();
    gpsMutex = xSemaphoreCreateMutex();
    loraQueue = xQueueCreate(10, sizeof(std::vector<int8_t> *));
    // core 0 system , core 1 user
    xTaskCreatePinnedToCore(_LoRaListenTask, "LoRaListenTask", 8192, NULL, 1, &Task5, 0); // xTaskCreatePinnedToCore(_BleCommunicationTask, "BleCommunicationTask", 2048, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(_LoRaSendTask, "LoRaSendTask", 2048, NULL, 1, &Task1, 0);     // xTaskCreatePinnedToCore(_BleCommunicationTask, "BleCommunicationTask", 2048, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(_GpsUpdateTask, "GpsUpdateTask", 4096, NULL, 1, &Task4, 1);

    xTaskCreatePinnedToCore(_ButtonPressTask, "ButtonPressTask", 2048, NULL, 2, &Task2, 1);
    xTaskCreatePinnedToCore(_OledDisplayTask, "OledDisplayTask", 4096, NULL, 1, &Task3, 1);
    xTaskCreatePinnedToCore(_SleepManagerTask, "SleepManagerTask", 4096, NULL, 2, &Task6, 1);
    // xTaskCreatePinnedToCore(_BackNavigationTask, "BackNavigationTask", 2048, nullptr, 2, nullptr, 0);

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
    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5)))
    {
        page = (page + 1) % 5; // Cycle through screens 0 to 4
        xSemaphoreGive(xSemaphore);
    }
}

void sleepBtnPressed()
{
    // blinkRED();
    Serial.println("Sleep button pressed");
}

void upBtnPressed()
{
    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5)))
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
    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5)))
    {
        int sizeofinbox = inboxBucketPtr->size();
        printf("Inbox size: %d\n", sizeofinbox);
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
    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5)))
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
            if (states[3][0]==1){
              states[3][1] = 1; // Set detail flag
            }else{
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
                        printStatesOfButtons();
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
        u8g2.drawStr(0, 10, "WELCOME"); // Display welcome message
        u8g2.drawLine(0, 11, 127, 11);  // Draw horizontal line below the text
    } while (u8g2.nextPage());
}

void renderInbox()
{
    u8g2.firstPage();
    do
    {   u8g2.clearBuffer();
        u8g2.setFont(H_FONT);
        u8g2.drawStr(0, 10, "INBOX");
        u8g2.drawLine(0, 11, 127, 11);

        u8g2.setFont(P_FONT);
        if (inboxBucketPtr->size() == 0)
        {
            u8g2.drawStr(0, 25, "No messages");
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
                u8g2.drawStr(2, 63, "OK → Open Msg");
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
        u8g2.drawStr(1, 10, "SEND");
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
                    u8g2.drawStr(2, y, PredefinedMessages[idx].c_str());
                    u8g2.setDrawColor(1); // restore
                }
                else
                {
                    // normal text
                    u8g2.drawStr(2, y, PredefinedMessages[idx].c_str());
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
                vTaskDelay(100 / portTICK_PERIOD_MS);
                
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
        u8g2.clearBuffer();
        u8g2.setFont(H_FONT);
        u8g2.drawStr(0, 10, "BACK NAVIGATION");
        u8g2.drawLine(0, 11, 127, 11);

        // If navigation already running, show indicator
        // if (states[3][1] == 1) {
        //     u8g2.setFont(P_FONT);
        //     u8g2.drawStr(84, 10, "Nav: ON");
        // }

        // Prompt text (two lines)
        u8g2.setFont(P_FONT);
        u8g2.drawStr(0, 25, "Do you want to start");
        u8g2.drawStr(0, 36, "back navigation?");

        // Option 0: "No" at y = 46
        if (states[3][0] == 0) {
            u8g2.drawBox(0, 38, 128, 10);   // highlight box
            u8g2.setDrawColor(0);
            u8g2.drawStr(6, 46, "No");
            u8g2.setDrawColor(1);
        } else {
            u8g2.drawStr(6, 46, "No");
        }

        // Option 1: "Yes" at y = 56
        if (states[3][0] == 1) {
            u8g2.drawBox(0, 48, 128, 10);   // highlight box
            u8g2.setDrawColor(0);
            u8g2.drawStr(6, 56, "Yes");
            u8g2.setDrawColor(1);
        } else {
            u8g2.drawStr(6, 56, "Yes");
        }

    } while (u8g2.nextPage());
}

void renderSent()
{
    u8g2.firstPage();
    do
    {   u8g2.clearBuffer();
        u8g2.setFont(H_FONT);
        u8g2.drawStr(0, 10, "SENT BOX");
        u8g2.drawLine(0, 11, 127, 11);

        u8g2.setFont(P_FONT);
        if (sentBoxBucketPtr->size() == 0)
        {
            u8g2.drawStr(0, 25, "No messages");
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
                    u8g2.drawStr(2, y, sentBoxBucketPtr->at(i).c_str());
                    u8g2.setDrawColor(1);
                }
                else
                {
                    u8g2.drawStr(2, y, sentBoxBucketPtr->at(i).c_str());
                }
            }
            // Show detail-flag if OK pressedxxx
            if (states[1][1])
            {
                u8g2.drawBox(0, 58, 128, 6);
                u8g2.setDrawColor(0);
                u8g2.drawStr(2, 63, "OK → Open Msg");
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
    // while (1)
    // {

    //     int packetSize = LoRa.parsePacket();
    //     if (packetSize)
    //     {
    //         Serial.println("[Payload Recived]");
    //         std::vector<int8_t> newReceivedPayload;
    //         RSSI = LoRa.packetRssi();
    //         while (LoRa.available())
    //         {
    //             newReceivedPayload.push_back(LoRa.read());
    //         }
    //         userDevice.receive(newReceivedPayload);
    //         userDevice.printPayload();
    //     }

    //     // Let other tasks run
    //     vTaskDelay(pdMS_TO_TICKS(10)); // 10ms pause to feed watchdog
    // }

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
        Serial.println("[*] GPS Update Task Running");
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
                Serial.print("Latitude: ");
                Serial.println(gps.location.lat(), 8);
                Serial.print("Longitude: ");
                Serial.println(gps.location.lng(), 8);
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
            userDevice.setSentboxNew(userDevice.getMsg(*receivedVec));
            // userDevice.printPayload();
            delete receivedVec; // free when done
            states[2][1] = 0;   // reset sending flag
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed
    }
}

void _SleepManagerTask(void *pvParameters) {
pinMode(SLEEP_BTN, INPUT_PULLDOWN);
pinMode(LORA_DIO0, INPUT);

while (1) {
    // For INPUT_PULLDOWN, button is HIGH when pressed
    if (digitalRead(SLEEP_BTN) == HIGH) {
        Serial.println("[SleepManager] Sleep button pressed → Going to Light Sleep...");

        // Configure wake-up sources
        esp_sleep_enable_ext0_wakeup((gpio_num_t)LORA_DIO0, 1);  // Wake on LoRa IRQ (DIO0 goes HIGH)
        esp_sleep_enable_ext1_wakeup(BIT(SLEEP_BTN), ESP_EXT1_WAKEUP_ANY_HIGH); // Wake on button press

        // Prepare LoRa module for low power (optional)
        LoRa.sleep();  // SX127x sleep mode

        // Enter Light Sleep
        esp_light_sleep_start();

        // Wakes here after interrupt
        Serial.println("[SleepManager] Woke up!");
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
}
}