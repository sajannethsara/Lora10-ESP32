// #include <Arduino.h>
// #include <LoRa.h>
// #include "Protocol.h"
// #include <LiquidCrystal_I2C.h>

// #define LCD_ADDRESS 0x27
// #define LCD_COLUMNS 16
// #define LCD_ROWS 2
// LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);
// bool backLightState = true; // Initial backlight state
// unsigned long lastActivityTime = 0;
// #define LCD_TIMEOUT 30000 // 30 seconds in milliseconds

// void _LoRaListenTask(void *pvParameters);
// void _LoraSendTask(void *pvParameters);
// void _LCDDisplayTask(void *pvParameters);

// TaskHandle_t Task1;
// TaskHandle_t Task2;

// #define OK_BTN 33
// // #define MODE_BTN 32
// #define UP_BTN 25
// #define DOWN_BTN 27
// // #define SLEEP_BTN 5

// #define GREEN_LED 27
// #define BLUE_LED 2

// void blinkGREEN();
// void blinkBLUE();

// #define LORA_SS 5
// #define LORA_RST 14
// #define LORA_DIO0 26
// int RSSI = -50;
// // Device ID: 13, Device Level: 2
// InterDevicePayload InterDevice(13, 2);
// QueueHandle_t ForwardQueue;
// SemaphoreHandle_t xSemaphore;

// void setup()
// {
//     Serial.begin(115200);
//     pinMode(GREEN_LED, OUTPUT);
//     pinMode(BLUE_LED, OUTPUT);

//     pinMode(UP_BTN, INPUT_PULLDOWN);
//     pinMode(DOWN_BTN, INPUT_PULLDOWN);
//     pinMode(OK_BTN, INPUT_PULLDOWN);
//     // pinMode(MODE_BTN, INPUT_PULLDOWN);
//     // pinMode(SLEEP_BTN, INPUT_PULLDOWN);

//     // digitalWrite(RED_LED, HIGH);
//     // digitalWrite(BLUE_LED, HIGH);

//     LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

//     if (!LoRa.begin(433E6))
//     {
//         Serial.println("LoRa init failed.");
//         while (1)
//             ; // Base
//     }
//         Serial.println("LoRa init succeeded.");
//     // LoRa.setSyncWord(0x12);
//     LoRa.setTxPower(15);
//     LoRa.setSpreadingFactor(13);

//     lcd.init();
//     lcd.backlight();
//     // LoRa.setSy1ncWord(0x12);
//     xSemaphore = xSemaphoreCreateMutex();
//     ForwardQueue = xQueueCreate(10, sizeof(int8_t));
//     if (!ForwardQueue)
//     {
//         Serial.println("Queue creation failed!");
//         while (1)
//             ;
//     }else{
//         Serial.println("ForwardQueue created successfully!");
//     }

//     xTaskCreatePinnedToCore(_LoRaListenTask, "LoRaListenTask", 8192, NULL, 1, &Task1, 0);
//     xTaskCreatePinnedToCore(_LoraSendTask, "LoraSendTask", 4096, NULL, 1, &Task2, 1);
//     xTaskCreatePinnedToCore(_LCDDisplayTask, "LCDDisplayTask", 2048, NULL, 1, NULL, 1);

// }

// void _LoRaListenTask(void *pvParameters)
// {
//     while (1)
//     {
//         int packetSize = LoRa.parsePacket();
//         if (packetSize)
//         {
//             std::vector<int8_t> newReceivedPayload;
//             RSSI = LoRa.packetRssi();

//             while (LoRa.available())
//             {
//                 newReceivedPayload.push_back(LoRa.read());
//             }
//             blinkGREEN();
//             // receive task ek hadann one, meka
//             if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
//             {
//                 bool valid = InterDevice.receive(newReceivedPayload);
//                 if (valid)
//                 {
//                     blinkBLUE();
//                     InterDevice.setPayloadForward(newReceivedPayload);
//                 }
//                 xSemaphoreGive(xSemaphore);
//             }
//         }
//         // Serial.println("Running Lora Listening Task");
//         vTaskDelay(100 / portTICK_PERIOD_MS);
//     }
// }

// void _LoraSendTask(void *pvParameters)
// {
//     int8_t counter;
//     while (1)
//     {
//         if (xQueueReceive(ForwardQueue, &counter, portMAX_DELAY) == pdPASS)
//         {
//             if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
//             {
//                 InterDevice.loraSend();
//                 xSemaphoreGive(xSemaphore);
//             }
//             blinkBLUE();
//         }
//     }
//     vTaskDelay(100 / portTICK_PERIOD_MS);
// }

// void _LCDDisplayTask(void *pvParameters)
// {

//     backLightState = false;

//     for (;;)
//     {
//         bool pressed = false;

//         if (digitalRead(UP_BTN) == HIGH ||
//             digitalRead(DOWN_BTN) == HIGH ||
//             digitalRead(OK_BTN) == HIGH)
//         {
//             pressed = true;
//             lastActivityTime = millis();

//             if (!backLightState)
//             {
//                 backLightState = true;
//                 lcd.backlight();
//             }
//         }

//         // Auto turn off LCD
//         if (backLightState && (millis() - lastActivityTime > LCD_TIMEOUT))
//         {
//             backLightState = false;
//             lcd.noBacklight();
//         }
//     }
// }

// void loop()
// {
// }

// // {"userId": 77,"type": "pmsg","data": 12}

// void blinkGREEN()
// {
//     digitalWrite(GREEN_LED, HIGH);
//     vTaskDelay(100 / portTICK_PERIOD_MS);
//     digitalWrite(GREEN_LED, LOW);
// }

// void blinkBLUE()
// {
//     digitalWrite(BLUE_LED, HIGH);
//     vTaskDelay(100 / portTICK_PERIOD_MS);
//     digitalWrite(BLUE_LED, LOW);
// }

#include <Arduino.h>
#include <LoRa.h>
#include "Protocol.h"
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
bool backLightState = true;
unsigned long lastActivityTime = 0;
#define LCD_TIMEOUT 30000 // 30 seconds

void _LoRaListenTask(void *pvParameters);
void _LoraSendTask(void *pvParameters);
void _UserMessageSendTask(void *pvParameters);
void _ButtonPressTask(void *pvParameters);
// void _LCDDisplayTask(void *pvParameters);

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;

// Pin definitions (fixed UP/DOWN swap)

#define OK_BTN 32
#define UP_BTN 25
#define DOWN_BTN 33

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
    {UP_BTN, "UP", 50, upBtnPressed, false, 0},
    {DOWN_BTN, "DOWN", 50, downBtnPressed, false, 0},
    {OK_BTN, "OK", 50, okBtnPressed, false, 0}};
const size_t BUTTON_COUNT = sizeof(buttons) / sizeof(buttons[0]);

#define GREEN_LED 27
#define BLUE_LED 2

#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 26

int RSSI = -50;
InterDevicePayload InterDevice(13, 2);

QueueHandle_t ForwardQueue;
QueueHandle_t UserMsgQueue;
SemaphoreHandle_t xSemaphore;

std::vector<std::string> pmsgListForUser = InterDevice.getPredefinedMessagesForUser();
const int MSG_COUNT = pmsgListForUser.size();

int8_t selectedMsgIndex = 0;
bool sendCommand = false;
bool showSentMsg = false;
unsigned long sentMsgTimestamp = 0;

void blinkGREEN();
void blinkBLUE();

void setup()
{
    Serial.begin(115200);

    for (size_t i = 0; i < BUTTON_COUNT; i++)
    {
        pinMode(buttons[i].pin, INPUT_PULLDOWN);

        // external pulldown present
        buttons[i].lastStable = (digitalRead(buttons[i].pin) == HIGH);
        buttons[i].lastChangeMs = millis();
    }
    Serial.println("Button setup done.");

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
    Serial.println("LoRa init succeeded.");

    lcd.init();
    lcd.begin(16, 2);
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Select Msg:");
    lcd.setCursor(0, 1);
    lcd.print(pmsgListForUser[selectedMsgIndex].c_str());

    xSemaphore = xSemaphoreCreateMutex();
    ForwardQueue = xQueueCreate(10, sizeof(int8_t));
    UserMsgQueue = xQueueCreate(10, sizeof(int8_t));

    if (!ForwardQueue || !UserMsgQueue)
    {
        Serial.println("Queue creation failed!");
        while (1)
            ;
    }

    xTaskCreatePinnedToCore(_LoRaListenTask, "LoRaListenTask", 8192, NULL, 1, &Task1, 0);
    xTaskCreatePinnedToCore(_LoraSendTask, "LoraSendTask", 4096, NULL, 1, &Task2, 1);
    xTaskCreatePinnedToCore(_UserMessageSendTask, "UserMsgSendTask", 4096, NULL, 1, &Task3, 1);
    // xTaskCreatePinnedToCore(_LCDDisplayTask, "LCDDisplayTask", 2048, NULL, 1, &Task4, 1);
    xTaskCreatePinnedToCore(_ButtonPressTask, "ButtonPressTask", 2048, NULL, 2, &Task4, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
}

void _LoRaListenTask(void *pvParameters)
{
    while (1)
    {
        int packetSize = LoRa.parsePacket();
        if (packetSize)
        {
            Serial.print("-------[LoRa Received]------- | ");
            Serial.println(packetSize);

            std::vector<int8_t> newReceivedPayload;
            RSSI = LoRa.packetRssi();
            // Serial.print("Packet RSSI: ");
            // Serial.println(RSSI);

            while (LoRa.available())
            {
                int8_t val = LoRa.read();
                newReceivedPayload.push_back(val);
            }
            blinkGREEN();

            if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
            {
                bool valid = InterDevice.receive(newReceivedPayload);
                Serial.print("Payload valid: ");
                Serial.println(valid ? "true" : "false");
                if (valid)
                {
                    blinkBLUE();
                    InterDevice.setPayloadForward(newReceivedPayload);
                    std::string jsonPayload = InterDevice.getJsonPayload();
                    InterDevice.loraSend();
                    // Serial.println("Payload forwarded.");
                    Serial.print("Forwarded JSON: ");
                    Serial.println(jsonPayload.c_str());
                    InterDevice.printAckBucket();
                    Serial.println("-----------------------------");
                }
                xSemaphoreGive(xSemaphore);
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void _LoraSendTask(void *pvParameters)
{
    int8_t counter;
    while (1)
    {
        if (xQueueReceive(ForwardQueue, &counter, portMAX_DELAY) == pdPASS)
        {
            if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
            {
                InterDevice.loraSend();
                xSemaphoreGive(xSemaphore);
            }
            blinkBLUE();
        }
    }
}

// UPDATED: Create and send structured payload compatible with base station
void _UserMessageSendTask(void *pvParameters)
{
    int8_t msgIndex;
    while (1)
    {
        if (xQueueReceive(UserMsgQueue, &msgIndex, portMAX_DELAY) == pdPASS)
        {
            Serial.print("Sending message index over LoRa: ");
            Serial.println(msgIndex);

            // Create a payload with the selected message index as a "pmsg"
            InterDevice.createPmsg(msgIndex); // 0 attempts for simplicity

            // Optionally add ACK or any other metadata if needed

            // Send the payload
            InterDevice.loraSend();
            Serial.println("SENT");
        }
        blinkBLUE();
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
                        blinkBLUE();
                        // beep();
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
// void _LCDDisplayTask(void *pvParameters)
// {
//     backLightState = true;
//     lastActivityTime = millis();

//     for (;;)
//     {
//         bool pressed = false;
//         bool upPressed = digitalRead(UP_BTN) == HIGH;
//         bool downPressed = digitalRead(DOWN_BTN) == HIGH;
//         bool okPressed = digitalRead(OK_BTN) == HIGH;

//         if (upPressed || downPressed || okPressed)
//         {
//             pressed = true;
//             lastActivityTime = millis();

//             if (!backLightState)
//             {
//                 backLightState = true;
//                 lcd.backlight();
//             }
//         }

//         if (pressed)
//         {
//             if (upPressed)
//             {
//                 selectedMsgIndex--;
//                 if (selectedMsgIndex < 0)
//                     selectedMsgIndex = MSG_COUNT - 1;
//                 Serial.print("Selected Message Index: ");
//                 Serial.println(selectedMsgIndex);
//                 vTaskDelay(200 / portTICK_PERIOD_MS);
//             }
//             else if (downPressed)
//             {
//                 selectedMsgIndex++;
//                 if (selectedMsgIndex >= MSG_COUNT)
//                     selectedMsgIndex = 0;
//                 Serial.print("Selected Message Index: ");
//                 Serial.println(selectedMsgIndex);
//                 vTaskDelay(200 / portTICK_PERIOD_MS);
//             }
//             else if (okPressed && !showSentMsg)
//             {
//                 Serial.print("OK Pressed. Sending message index: ");
//                 Serial.println(selectedMsgIndex);
//                 xQueueSend(UserMsgQueue, &selectedMsgIndex, 0);

//                 lcd.clear();
//                 lcd.print("Sent:");
//                 lcd.setCursor(0, 1);
//                 lcd.print(pmsgListForUser[selectedMsgIndex].c_str());

//                 Serial.print("Sent: ");
//                 Serial.println(pmsgListForUser[selectedMsgIndex].c_str());

//                 showSentMsg = true;
//                 sentMsgTimestamp = millis();
//                 vTaskDelay(200 / portTICK_PERIOD_MS);
//             }
//         }

//         if (showSentMsg && millis() - sentMsgTimestamp >= 2000)
//         {
//             showSentMsg = false;
//             lcd.clear();
//         }

//         if (!showSentMsg)
//         {
//             lcd.setCursor(0, 0);
//             lcd.print("Select Msg:     ");
//             lcd.setCursor(0, 1);
//             lcd.print("                ");
//             lcd.setCursor(0, 1);
//             lcd.print(pmsgListForUser[selectedMsgIndex].c_str());
//         }

//         if (backLightState && (millis() - lastActivityTime > LCD_TIMEOUT))
//         {
//             backLightState = false;
//             lcd.noBacklight();
//         }

//         vTaskDelay(50 / portTICK_PERIOD_MS);
//     }
// }

void loop() {}

void blinkGREEN()
{
    digitalWrite(GREEN_LED, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(GREEN_LED, LOW);
}

void blinkBLUE()
{
    digitalWrite(BLUE_LED, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(BLUE_LED, LOW);
}

void upBtnPressed()
{
    selectedMsgIndex--;
    if (selectedMsgIndex < 0)
        selectedMsgIndex = MSG_COUNT - 1;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Select Msg:");
    lcd.setCursor(0, 1);
    lcd.print(pmsgListForUser[selectedMsgIndex].c_str());
    Serial.print("UP pressed. Selected Message Index: ");
    Serial.println(selectedMsgIndex);
}

void downBtnPressed()
{
    selectedMsgIndex++;
    if (selectedMsgIndex >= MSG_COUNT)
        selectedMsgIndex = 0;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Select Msg:");
    lcd.setCursor(0, 1);
    lcd.print(pmsgListForUser[selectedMsgIndex].c_str());
    Serial.print("DOWN pressed. Selected Message Index: ");
    Serial.println(selectedMsgIndex);
}

void okBtnPressed()
{
    xQueueSend(UserMsgQueue, &selectedMsgIndex, 0);
    lcd.clear();
    lcd.print("Sent:");
    lcd.setCursor(0, 1);
    lcd.print(pmsgListForUser[selectedMsgIndex].c_str());
    Serial.print("OK pressed. Sent: ");
    Serial.println(pmsgListForUser[selectedMsgIndex].c_str());
}