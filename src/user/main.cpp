// This is the final program - dont change this.
#include "Protocol.h"
#include <Arduino.h>
#include <LoRa.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>

TaskHandle_t Task1;
TaskHandle_t Task2;
void _LoRaListenTask(void *pvParameters);
void _ButtonPressTask(void *pvParameters);

// Pins
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 26
int RSSI = -50;

#define OK_BTN 32
#define UP_BTN 25
#define DOWN_BTN 33
#define MODE_BTN 35
#define SLEEP_BTN 36

#define RED_LED 27
#define BLUE_LED 2

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;

// Fonts
#define H_FONT u8g2_font_ncenB08_tr
#define P_FONT u8g2_font_6x10_tr

UserDevicePayload userDevice(77); // Device ID 77
int page = 0;
int states[5][2] = {0};
SemaphoreHandle_t xSemaphore;

void setup()
{
    Serial.begin(115200);
    // u8g2.begin();
    // u8g2.clearBuffer();

    pinMode(MODE_BTN, INPUT_PULLDOWN);
    pinMode(UP_BTN, INPUT_PULLDOWN);
    pinMode(DOWN_BTN, INPUT_PULLDOWN);
    pinMode(OK_BTN, INPUT_PULLDOWN);
    pinMode(SLEEP_BTN, INPUT_PULLDOWN);

    pinMode(RED_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);

    digitalWrite(RED_LED, HIGH);
    digitalWrite(BLUE_LED, HIGH);

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
    xTaskCreatePinnedToCore(_LoRaListenTask, "LoRaListenTask", 8192, NULL, 1, &Task1, 0);    // xTaskCreatePinnedToCore(_BleCommunicationTask, "BleCommunicationTask", 2048, nullptr, 1, nullptr, 1);
    // xTaskCreatePinnedToCore(_GpsUpdateTask, "GpsUpdateTask", 2048, nullptr, 1, nullptr, 1);

    xTaskCreatePinnedToCore(_ButtonPressTask, "ButtonPressTask", 2048, NULL, 1, &Task2, 1);
    // xTaskCreatePinnedToCore(_OledDisplayTask, "OledDisplayTask", 4096, nullptr, 1, nullptr, 0);
    // xTaskCreatePinnedToCore(_BackNavigationTask, "BackNavigationTask", 2048, nullptr, 2, nullptr, 0);

    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
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
    Serial.println("Mode button pressed");
}

void sleepBtnPressed()
{
    // Sleeping Logic Should Apply
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
        Serial.println("Up button pressed");
    }
}

void downBtnPressed()
{
    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5)))
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
        Serial.println("Down button pressed");
    }
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
    }
    Serial.println("OK button pressed");
}
void printStatesOfButtons(){
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
    Serial.println("Button Press Task Started");
    while (1)
    {
        if (millis() - lastDebounceTime > debounceDelay)
        {
            if (digitalRead(MODE_BTN) == HIGH)
            {
                Serial.println("MODE");
                modeBtnPressed();
                printStatesOfButtons();
                lastDebounceTime = millis();
            }

            if (digitalRead(UP_BTN) == HIGH)
            {
                Serial.println("UP");
                upBtnPressed();
                printStatesOfButtons();
                lastDebounceTime = millis();
            }

            if (digitalRead(DOWN_BTN) == HIGH)
            {
                Serial.println("DOWN");
                downBtnPressed();
                printStatesOfButtons();
                lastDebounceTime = millis();
            }

            if (digitalRead(OK_BTN) == HIGH)
            {
                Serial.println("OK");
                okBtnPressed();
                printStatesOfButtons();
                lastDebounceTime = millis();
            }

            if (digitalRead(SLEEP_BTN) == HIGH)
            {
                Serial.println("SLEEP");
                sleepBtnPressed();
                printStatesOfButtons();
                lastDebounceTime = millis();
            }
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}




// // OLED
// void renderWelcome()
// {
//     u8g2.firstPage();
//     do
//     {
//         u8g2.setFont(H_FONT);                       // Set header font
//         u8g2.drawStr(0, 10, "WELCOME");             // Display welcome message
//         u8g2.drawLine(0, 11, 127, 11);              // Draw horizontal line below the text
//     } while (u8g2.nextPage());
// }

// void renderInbox()
// {
//     u8g2.firstPage();
//     do
//     {
//         u8g2.setFont(H_FONT);
//         u8g2.drawStr(0, 10, "INBOX");
//         u8g2.drawLine(0, 11, 127, 11);

//         u8g2.setFont(P_FONT);
//         if (messageCount == 0)
//         {
//             u8g2.drawStr(0, 25, "No messages");
//         }
//         else
//         {
//             for (int i = 0; i < messageCount && i < 5; i++)
//             {
//                 int y = 25 + i * 10;
//                 if (i == states[1][0])
//                 {
//                     u8g2.drawBox(0, y - 8, 128, 10);
//                     u8g2.setDrawColor(0);
//                     u8g2.drawStr(2, y, receivedMessages[i].messageContent.c_str());
//                     u8g2.setDrawColor(1);
//                 }
//                 else
//                 {
//                     u8g2.drawStr(2, y, receivedMessages[i].messageContent.c_str());
//                 }
//             }
//             // Show detail-flag if OK pressed
//             if (states[1][1])
//             {
//                 u8g2.drawBox(0, 58, 128, 6);
//                 u8g2.setDrawColor(0);
//                 u8g2.drawStr(2, 63, "OK → Open Msg");
//                 u8g2.setDrawColor(1);
//             }
//         }
//     } while (u8g2.nextPage());
// }

// void renderSend()
// {
//     u8g2.firstPage();
//     do
//     {
//         u8g2.setFont(H_FONT);
//         u8g2.drawStr(0, 10, "SEND");
//         u8g2.drawLine(0, 11, 127, 11);

//         u8g2.setFont(P_FONT);
//         u8g2.drawStr(0, 24, "Selected Msg:");
//         const char* txt = emergencyMessages[states[2][0]];
//         u8g2.drawStr(0, 36, txt);

//         // “sending” indicator
//         if (states[2][1])
//         {
//             u8g2.drawStr(0, 50, "Sending...");
//         }
//     } while (u8g2.nextPage());
// }

// void renderCompass()
// {
//     u8g2.firstPage();
//     do
//     {
//         // Oled UI Logic
//     } while (u8g2.nextPage());
// }

// void renderBluetooth()
// {
//     u8g2.firstPage();
//     do
//     {
//         // Oled UI Logic
//     } while (u8g2.nextPage());
// }

// void _OledDisplayTask(void *pvParameters)
// {
//     while (1)
//     {
//         if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
//         {
//             switch (page)
//             {
//             case 0:
//                 renderWelcome();
//                 break;
//             case 1:
//                 renderInbox();
//                 break;
//             case 2:
//                 renderSend();
//                 break;
//             case 3:
//                 renderCompass();
//                 break;
//             case 4:
//                 renderBluetooth();
//                 break;
//             }
//             xSemaphoreGive(xSemaphore);
//         }
//         vTaskDelay(100 / portTICK_PERIOD_MS);
//     }
// }

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
