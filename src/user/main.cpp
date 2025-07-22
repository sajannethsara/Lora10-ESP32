// This is the final program - dont change this.
#include "Protocol.h"
#include <Arduino.h>
#include <LoRa.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
// Pins
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 26
int RSSI = -50;

#define OK_BTN 33
#define MODE_BTN 32
#define UP_BTN 25
#define DOWN_BTN 27
#define SLEEP_BTN 5
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;

// Fonts
#define H_FONT u8g2_font_ncenB08_tr
#define P_FONT u8g2_font_6x10_tr

UserDevicePayload userDevice(53); // Device ID 53
int page = 0;
int states[5][2] = {0};
SemaphoreHandle_t xSemaphore;

void setup()
{
    Serial.begin(115200);
    u8g2.begin();
    u8g2.clearBuffer();

    pinMode(MODE_BTN, INPUT_PULLDOWN);
    pinMode(UP_BTN, INPUT_PULLDOWN);
    pinMode(DOWN_BTN, INPUT_PULLDOWN);
    pinMode(OK_BTN, INPUT_PULLDOWN);
    pinMode(SLEEP_BTN, INPUT_PULLDOWN);

    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if (!LoRa.begin(433E6))
    {
        Serial.println("LoRa init failed.");
        while (1)
            ;
    }
    Serial.println("LoRa init succeeded.");
    // LoRa.setSyncWord(0x12);
    LoRa.setTxPower(15);
    LoRa.setSpreadingFactor(13);

    xSemaphore = xSemaphoreCreateMutex();

    // core 0 system , core 1 user
    xTaskCreatePinnedToCore(_LoRaListenTask, "LoRaListenTask", 2048, nullptr, 2, nullptr, 1);
    // xTaskCreatePinnedToCore(_BleCommunicationTask, "BleCommunicationTask", 2048, nullptr, 1, nullptr, 1);
    // xTaskCreatePinnedToCore(_GpsUpdateTask, "GpsUpdateTask", 2048, nullptr, 1, nullptr, 1);

    xTaskCreatePinnedToCore(_ButtonPressTask, "ButtonPressTask", 2048, nullptr, 1, nullptr, 0);
    xTaskCreatePinnedToCore(_OledDisplayTask, "OledDisplayTask", 4096, nullptr, 1, nullptr, 0);
    // xTaskCreatePinnedToCore(_BackNavigationTask, "BackNavigationTask", 2048, nullptr, 2, nullptr, 0);
}
// BTN
void modeBtnPressed()
{
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
    {
        page = (page + 1) % 5; // Cycle through screens 0 to 4
        xSemaphoreGive(xSemaphore);
    }
}

void sleepBtnPressed()
{
    // Sleeping Logic Should Apply
}

void upBtnPressed()
{
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
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
    }
}

void downBtnPressed()
{
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
    {
        if (page == 1 && states[1][0] < 9) // Inbox
        {
            states[1][0]++;
        }
        else if (page == 2 && states[2][0] > 0) // Send
        {
            states[2][0]--;
        }
        xSemaphoreGive(xSemaphore);
    }
}

void okBtnPressed()
{
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
    {
        if (page == 2) // Send
        {
            states[2][1] = states[2][0];                         // Set selected message to send
            states[2][2] = 1;                                    // Indicate sending state
            xQueueSend(loraQueue, &states[2][1], portMAX_DELAY); // Send message ID to LoRa task
        }
        else if (page == 1) // Inbox
        {
            states[1][1] = 1; // Set OK flag for inbox
        }
        xSemaphoreGive(xSemaphore);
    }
}

void _ButtonPressTask(void *pvParameters)
{
    while (1)
    {
        if (millis() - lastDebounceTime > debounceDelay)
        {
            if (digitalRead(MODE_BTN) == HIGH)
            {
                modeBtnPressed();
                lastDebounceTime = millis();
            }

            if (digitalRead(UP_BTN) == HIGH)
            {
                upBtnPressed();
                lastDebounceTime = millis();
            }

            if (digitalRead(DOWN_BTN) == HIGH)
            {
                downBtnPressed();
                lastDebounceTime = millis();
            }

            if (digitalRead(OK_BTN) == HIGH)
            {
                okBtnPressed();
                lastDebounceTime = millis();
            }

            if (digitalRead(SLEEP_BTN) == HIGH)
            {
                sleepBtnPressed();
                lastDebounceTime = millis();
            }
            
            // ----- Test Output -----
            if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
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
            // ------------------------
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// OLED
void renderInbox()
{
    u8g2.firstPage();
    do
    {
        // Oled UI Logic
    } while (u8g2.nextPage());
}

void renderSend()
{
    u8g2.firstPage();
    do
    {
        // Oled UI Logic
    } while (u8g2.nextPage());
}

void renderCompass()
{
    u8g2.firstPage();
    do
    {
        // Oled UI Logic
    } while (u8g2.nextPage());
}

void renderBluetooth()
{
    u8g2.firstPage();
    do
    {
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
            std::vector<uint8_t> newReceivedPayload;
            RSSI = LoRa.packetRssi();
            while (LoRa.available())
            {
                newReceivedPayload.push_back(LoRa.read());
            }
            userDevice.receive(newReceivedPayload);
            
        }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

