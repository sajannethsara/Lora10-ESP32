#include <Arduino.h>
#include <LoRa.h>
#include "Protocol.h"
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
bool backLightState = true; // Initial backlight state
unsigned long lastActivityTime = 0;
#define LCD_TIMEOUT 30000 // 30 seconds in milliseconds

void _LoRaListenTask(void *pvParameters);
void _LoraSendTask(void *pvParameters);
void _LCDDisplayTask(void *pvParameters);

TaskHandle_t Task1;
TaskHandle_t Task2;

#define OK_BTN 33
// #define MODE_BTN 32
#define UP_BTN 25
#define DOWN_BTN 27
// #define SLEEP_BTN 5

#define GREEN_LED 27
#define BLUE_LED 2

void blinkGREEN();
void blinkBLUE();

#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 26
int RSSI = -50;
// Device ID: 13, Device Level: 2
InterDevicePayload InterDevice(13, 2);
QueueHandle_t ForwardQueue;
SemaphoreHandle_t xSemaphore;

void setup()
{
    Serial.begin(115200);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);

    pinMode(UP_BTN, INPUT_PULLDOWN);
    pinMode(DOWN_BTN, INPUT_PULLDOWN);
    pinMode(OK_BTN, INPUT_PULLDOWN);
    // pinMode(MODE_BTN, INPUT_PULLDOWN);
    // pinMode(SLEEP_BTN, INPUT_PULLDOWN);

    // digitalWrite(RED_LED, HIGH);
    // digitalWrite(BLUE_LED, HIGH);

    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(433E6))
    {
        Serial.println("LoRa init failed.");
        while (1)
            ; // Base
    }
        Serial.println("LoRa init succeeded.");
    // LoRa.setSyncWord(0x12);
    LoRa.setTxPower(15);
    LoRa.setSpreadingFactor(13);


    lcd.init();
    lcd.backlight();
    // LoRa.setSy1ncWord(0x12);
    xSemaphore = xSemaphoreCreateMutex();
    ForwardQueue = xQueueCreate(10, sizeof(int8_t));
    if (!ForwardQueue)
    {
        Serial.println("Queue creation failed!");
        while (1)
            ;
    }else{
        Serial.println("ForwardQueue created successfully!");
    }

    xTaskCreatePinnedToCore(_LoRaListenTask, "LoRaListenTask", 8192, NULL, 1, &Task1, 0);
    xTaskCreatePinnedToCore(_LoraSendTask, "LoraSendTask", 4096, NULL, 1, &Task2, 1);
    xTaskCreatePinnedToCore(_LCDDisplayTask, "LCDDisplayTask", 2048, NULL, 1, NULL, 1);
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
            std::vector<int8_t> newReceivedPayload;
            RSSI = LoRa.packetRssi();

            while (LoRa.available())
            {
                newReceivedPayload.push_back(LoRa.read());
            }
            blinkGREEN();
            // receive task ek hadann one, meka
            if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
            {
                bool valid = InterDevice.receive(newReceivedPayload);
                if (valid)
                {
                    blinkBLUE();
                    InterDevice.setPayloadForward(newReceivedPayload);
                }
                xSemaphoreGive(xSemaphore);
            }
        }
        // Serial.println("Running Lora Listening Task");
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
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void _LCDDisplayTask(void *pvParameters)
{

    backLightState = false;

    for (;;)
    {
        bool pressed = false;

        if (digitalRead(UP_BTN) == HIGH ||
            digitalRead(DOWN_BTN) == HIGH ||
            digitalRead(OK_BTN) == HIGH)
        {
            pressed = true;
            lastActivityTime = millis();

            if (!backLightState)
            {
                backLightState = true;
                lcd.backlight();
            }
        }

        // Auto turn off LCD
        if (backLightState && (millis() - lastActivityTime > LCD_TIMEOUT))
        {
            backLightState = false;
            lcd.noBacklight();
        }
    }
}

void loop()
{
}

// {"userId": 77,"type": "pmsg","data": 12}

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
