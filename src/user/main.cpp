// This is the final program - dont change this.
#include "Protocol.h"
#include <Arduino.h>
#include <LoRa.h>
#include <U8g2lib.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <SPI.h>

UserDevicePayload userDevice(77); // Device ID 77

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;

void _LoRaListenTask(void *pvParameters);
void _ButtonPressTask(void *pvParameters);
void _OledDisplayTask(void *pvParameters);
void _GpsUpdateTask(void *pvParameters);

// Pins
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 26
int RSSI = -50;

//------BUTTONS---------
#define OK_BTN 32
#define UP_BTN 25
#define DOWN_BTN 33
#define MODE_BTN 35
#define SLEEP_BTN 36

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
//----------------------

#define RED_LED 27
#define BLUE_LED 2
void blinkRED();
void blinkBLUE();

// OLED
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
#define H_FONT u8g2_font_ncenB08_tr
#define P_FONT u8g2_font_6x10_tr

// GPS
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
HardwareSerial GPS_Serial(2);
TinyGPSPlus gps;
float latitude = 0.0;
float longitude = 0.0;
bool gpsFix = false;
SemaphoreHandle_t gpsMutex;

void setup()
{
    Serial.begin(115200);
    Serial.println("[*] Starting Device... [Serial:115200]");

    u8g2.begin();
    u8g2.clearBuffer();

    pinMode(RED_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);
    digitalWrite(RED_LED, HIGH);
    Serial.println("[+] LEDs initialized.");

    for (size_t i = 0; i < BUTTON_COUNT; i++)
    {
        pinMode(buttons[i].pin, INPUT_PULLDOWN); // external pulldown present
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
    LoRa.setTxPower(15);
    LoRa.setSpreadingFactor(13);
    Serial.println("[+] LoRa init succeeded.");

    xSemaphore = xSemaphoreCreateMutex();

    // core 0 system , core 1 user
    xTaskCreatePinnedToCore(_LoRaListenTask, "LoRaListenTask", 8192, NULL, 1, &Task1, 0); // xTaskCreatePinnedToCore(_BleCommunicationTask, "BleCommunicationTask", 2048, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(_GpsUpdateTask, "GpsUpdateTask", 2048, NULL, 1, &Task4, 1);

    xTaskCreatePinnedToCore(_ButtonPressTask, "ButtonPressTask", 2048, NULL, 1, &Task2, 1);
    xTaskCreatePinnedToCore(_OledDisplayTask, "OledDisplayTask", 4096, NULL, 1, &Task3, 1);
    // xTaskCreatePinnedToCore(_BackNavigationTask, "BackNavigationTask", 2048, nullptr, 2, nullptr, 0);

    digitalWrite(BLUE_LED, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
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
    blinkRED();
    Serial.println("Mode button pressed");
}

void sleepBtnPressed()
{
    blinkRED();
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
        else if (page == 2 && states[2][0] < 9) // Send
        {
            states[2][0]++;
        }
        xSemaphoreGive(xSemaphore);
        blinkBLUE();
        Serial.println("Up button pressed");
    }
}

void downBtnPressed()
{
    if (page == 0)
    {
        states[0][0]++;
    }
    if (page == 1 && states[1][0] < 10) // Inbox
    {
        states[1][0]++;
    }
    else if (page == 2 && states[2][0] > 0) // Send
    {
        states[2][0]--;
    }
    else if (page == 3) // Compass
    {
        states[3][0]++;
    }
    else if (page == 4) // Bluetooth
    {
        states[4][0]++;
    }
    blinkBLUE();
    Serial.println("Down button pressed");
}

void okBtnPressed()
{
    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5)))
    {
        if (page == 1) // Inbox
        {
            states[1][1] = 1; // Set OK flag for inbox
        }
        else if (page == 2) // Send
        {
            int selectedMsg = states[2][0]; // selection index
            // xQueueSend(loraQueue, &selectedMsg, portMAX_DELAY); // send selection
            states[2][1] = 1; // flag as "sending"
        }
        xSemaphoreGive(xSemaphore);
        blinkBLUE();
    }
    Serial.println("OK button pressed");
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
        u8g2.setFont(H_FONT);           // Set header font
        u8g2.drawStr(0, 10, "WELCOME"); // Display welcome message
        u8g2.drawLine(0, 11, 127, 11);  // Draw horizontal line below the text
    } while (u8g2.nextPage());
}

void renderInbox()
{
    u8g2.firstPage();
    do
    {
        u8g2.setFont(H_FONT);
        u8g2.drawStr(0, 10, "INBOX");
        u8g2.drawLine(0, 11, 127, 11);

        // u8g2.setFont(P_FONT);
        // if (messageCount == 0)
        // {
        //     u8g2.drawStr(0, 25, "No messages");
        // }
        // else
        // {
        //     for (int i = 0; i < messageCount && i < 5; i++)
        //     {
        //         int y = 25 + i * 10;
        //         if (i == states[1][0])
        //         {
        //             u8g2.drawBox(0, y - 8, 128, 10);
        //             u8g2.setDrawColor(0);
        //             u8g2.drawStr(2, y, receivedMessages[i].messageContent.c_str());
        //             u8g2.setDrawColor(1);
        //         }
        //         else
        //         {
        //             u8g2.drawStr(2, y, receivedMessages[i].messageContent.c_str());
        //         }
        //     }
        //     // Show detail-flag if OK pressed
        //     if (states[1][1])
        //     {
        //         u8g2.drawBox(0, 58, 128, 6);
        //         u8g2.setDrawColor(0);
        //         u8g2.drawStr(2, 63, "OK → Open Msg");
        //         u8g2.setDrawColor(1);
        //     }
        // }
    } while (u8g2.nextPage());
}

void renderSend()
{
    u8g2.firstPage();
    do
    {
        u8g2.setFont(H_FONT);
        u8g2.drawStr(0, 10, "SEND");
        u8g2.drawLine(0, 11, 127, 11);

        // u8g2.setFont(P_FONT);
        // u8g2.drawStr(0, 24, "Selected Msg:");
        // const char *txt = emergencyMessages[states[2][0]];
        // u8g2.drawStr(0, 36, txt);

        // // “sending” indicator
        // if (states[2][1])
        // {
        //     u8g2.drawStr(0, 50, "Sending...");
        // }
    } while (u8g2.nextPage());
}

void renderCompass()
{
    u8g2.firstPage();
    do
    {
        u8g2.setFont(H_FONT);
        u8g2.drawStr(0, 10, "COMPASS");
        u8g2.drawLine(0, 11, 127, 11);
        // Oled UI Logic
    } while (u8g2.nextPage());
}

void renderBluetooth()
{
    u8g2.firstPage();
    do
    {
        u8g2.setFont(H_FONT);
        u8g2.drawStr(0, 10, "BLE");
        u8g2.drawLine(0, 11, 127, 11);
        // Oled UI Logic
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
                renderBluetooth();
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
            std::vector<int8_t> newReceivedPayload;
            RSSI = LoRa.packetRssi();
            while (LoRa.available())
            {
                newReceivedPayload.push_back(LoRa.read());
            }
            // userDevice.receive(newReceivedPayload);
            userDevice.setPayload(newReceivedPayload);
            userDevice.printPayload();
        }

        // Let other tasks run
        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms pause to feed watchdog
    }
}

void _GpsUpdateTask(void *parameter)
{
    while (1)
    {
        while (GPS_Serial.available() > 0)
        {
            char c = GPS_Serial.read();
            gps.encode(c);

            if (gps.location.isUpdated())
            {
                xSemaphoreTake(gpsMutex, portMAX_DELAY);
                latitude = gps.location.lat();
                longitude = gps.location.lng();
                gpsFix = gps.location.isValid();
                Serial.print("Lat: ");
                Serial.println(latitude, 6);
                Serial.print("Lon: ");
                Serial.println(longitude, 6);
                Serial.print("Fix: ");
                Serial.println(gpsFix ? "Yes" : "No");
                xSemaphoreGive(gpsMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
} 

void blinkBLUE()
{
    digitalWrite(BLUE_LED, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(BLUE_LED, LOW);
}

void blinkRED()
{
    digitalWrite(RED_LED, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(RED_LED, LOW);
}