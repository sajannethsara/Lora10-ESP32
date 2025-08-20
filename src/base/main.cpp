#include <Arduino.h>
#include <LoRa.h>
#include "Protocol.h"
#include <ArduinoJson.h>

void _LoRaListenTask(void *pvParameters);
void _LoRaSendTask(void *pvParameters);
void _SerialListenTask(void *pvParameters);
void _LCDTask(void *pvParameters);

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;

#define RED_LED 02
#define BLUE_LED 27
#define BUZZER_PIN 4
void blinkGREEN();
void blinkBLUE();

#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 26
int RSSI = -50;

BaseDevicePayload baseDevice(0);
SemaphoreHandle_t xSemaphore;
QueueHandle_t loraQueue;
std::string mostRecentMsg;
int8_t mostRecentUid;
void setMostRecentMsg(const std::string &msg);

void setup()
{
    Serial.begin(115200);
    pinMode(RED_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
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

    loraQueue = xQueueCreate(10, sizeof(std::vector<int8_t> *));
    xSemaphore = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(_LoRaListenTask, "LoRaListenTask", 8192, NULL, 1, &Task1, 0);
    xTaskCreatePinnedToCore(_SerialListenTask, "SerialListenTask", 4096, NULL, 2, &Task2, 1);
    xTaskCreatePinnedToCore(_LoRaSendTask, "LoRaSendTask", 2048, NULL, 3, &Task3, 1);
    xTaskCreatePinnedToCore(_LCDTask, "LCDTask", 2048, NULL, 1, &Task4, 1);

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
            Serial.println("[Received packet]");
            std::vector<int8_t> newReceivedPayload;
            std::string jsonPayload;
            RSSI = LoRa.packetRssi();
            while (LoRa.available())
            {
                newReceivedPayload.push_back(LoRa.read());
            }

            bool valid = baseDevice.receive(newReceivedPayload);
            baseDevice.setPayload(newReceivedPayload);
            std::string msg = baseDevice.getMsg(*newReceivedPayload);
            int8_t uid = baseDevice.getUid(*newReceivedPayload);
            if (valid)
            {   
                setMostRecentMsg(*msg,uid);
                jsonPayload = baseDevice.getJsonPayload();
                Serial.println("[Payload Received]");
                Serial.println(jsonPayload.c_str());
                // baseDevice.printAckBucket();
            }
            // handleReceivedPayload(receivedPayload);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Check for new LoRa messages at a regular interval
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
            DeserializationError error = deserializeJson(doc, input.c_str());

            Serial.print("Received from Base Station : ");
            Serial.println(input.c_str());

            if (!error)
            {
                int userId = doc["userId"] | -1;

                if (doc["data"].is<int>())
                {
                    Serial.println("PMSG");
                    int numberData = doc["data"];
                    baseDevice.createPmsg(numberData, userId, 0);
                    std::vector<int8_t> *payloadVector = new std::vector<int8_t>(baseDevice.getPayload());
                    xQueueSend(loraQueue, &payloadVector, portMAX_DELAY);
                }
                else if (doc["data"].is<const char *>())
                {
                    Serial.println("CMSG");
                    std::string stringData = doc["data"].as<const char *>();
                    baseDevice.createCmsg(stringData, userId, 0);
                    std::vector<int8_t> *payloadVector = new std::vector<int8_t>(baseDevice.getPayload());
                    xQueueSend(loraQueue, &payloadVector, portMAX_DELAY);
                }
                else
                {
                    Serial.println("Unknown data type!");
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

void _LoRaSendTask(void *pvParameters)
{
    for (;;)
    {
        std::vector<int8_t> *receivedVec;
        if (xQueueReceive(loraQueue, &receivedVec, portMAX_DELAY))
        {
            bool pass = baseDevice.setPayload(*receivedVec);
            printf("Payload set: %s\n", pass ? "[true]" : "[false]");
            baseDevice.loraSend();
            delete receivedVec; // free when done
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed
    }
}

void _LCDTask(void *pvParameters)
{
    
}

void loop() {}

// {"userId": 77,"type": "pmsg","data": 12}
// {"userId": 77,"type": "cmsg","data": "Hi man raju"}

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
void setMostRecentMsg(const std::string &msg,int8_t uid){
    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5)))
    {
        mostRecentMsg = msg;
        mostRecentUid = uid;
        xSemaphoreGive(xSemaphore);
    }
}