#include <Arduino.h>
#include <LoRa.h>
#include "Protocol.h"
#include <ArduinoJson.h>

void _LoRaListenTask(void *pvParameters);
void _SerialListenTask(void *pvParameters);

TaskHandle_t Task1;
TaskHandle_t Task2;   

#define RED_LED 02
#define BLUE_LED 27
void blinkGREEN();
void blinkBLUE();

#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 26
int RSSI = -50;

BaseDevicePayload baseDevice(0);

void setup()
{
    Serial.begin(115200);
    pinMode(RED_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);

    digitalWrite(RED_LED, HIGH);
    digitalWrite(BLUE_LED, HIGH);

    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0); 
    LoRa.setTxPower(17);
    LoRa.setSpreadingFactor(7);
    if (!LoRa.begin(433E6))
    {
        Serial.println("LoRa init failed.");
        while (1)
            ;
    }
    Serial.println("LoRa init succeeded.");
    // LoRa.setSy1ncWord(0x12);

    xTaskCreatePinnedToCore(_LoRaListenTask, "LoRaListenTask", 8192, NULL, 1, &Task1, 0);
    xTaskCreatePinnedToCore(_SerialListenTask, "SerialListenTask", 4096, NULL, 1, &Task2, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
}

void _LoRaListenTask(void *pvParameters)
{
    while (1)
    {
        int packetSize = LoRa.parsePacket();
        if (packetSize)
        {
            Serial.print("Received packet: ");
            std::vector<int8_t> newReceivedPayload;
            std::string jsonPayload;
            RSSI = LoRa.packetRssi();
            while (LoRa.available())
            {
                newReceivedPayload.push_back(LoRa.read());
            }
            blinkGREEN();
            // receive task ek hadann one, meka
            blinkGREEN();
            Serial.println("[*] Received a packet : ");
            for (const auto &byte : newReceivedPayload)
            {
                Serial.print(byte, HEX);
                Serial.print(" ");
            }
            Serial.println();
            bool valid = baseDevice.receive(newReceivedPayload);
            baseDevice.setPayload(newReceivedPayload);
            if (valid)
            {   
                blinkBLUE();
                jsonPayload = baseDevice.getJsonPayload();
                Serial.println("[Payload Received]");
                Serial.println(jsonPayload.c_str());
                baseDevice.printAckBucket();
            }
        }
        // Serial.println("Running Lora Listening Task");
        vTaskDelay(pdMS_TO_TICKS(1)); // 1ms pause to feed watchdog
    }
}

void _SerialListenTask(void *pvParameters)
{
    JsonDocument doc;

    while (1)
    {
        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n');
            DeserializationError error = deserializeJson(doc, input);

            Serial.print("Received from Base Station : ");
            Serial.println(input.c_str());

            if (!error)
            {
                int userId = doc["userId"] | -1;

                if (doc["data"].is<int>())
                { 
                    int numberData = doc["data"];
                    baseDevice.createPmsg(numberData, userId, 0);
                    std::vector<int8_t> payloadVector = baseDevice.getPayload();
                    baseDevice.addAck(payloadVector);
                    baseDevice.loraSend();
                    blinkBLUE();
                }
                else if (doc["data"].is<const char *>())
                { 
                    std::string stringData = doc["data"].as<const char *>();
                    baseDevice.createCmsg(stringData, userId, 0);
                    std::vector<int8_t> payloadVector = baseDevice.getPayload();
                    baseDevice.addAck(payloadVector);
                    baseDevice.loraSend();
                    blinkBLUE();
                }
            }
            else
            {
                Serial.println("JSON Parse Error!");
            }
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}


void loop(){}

// {"userId": 77,"type": "pmsg","data": 12}
// {"userId": 77,"type": "cmsg","data": "Hi man raju"}


void blinkGREEN()
{
    digitalWrite(RED_LED, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(RED_LED, LOW);
}

void blinkBLUE()
{
    digitalWrite(BLUE_LED, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(BLUE_LED, LOW);
}


