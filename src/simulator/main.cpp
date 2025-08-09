// #include <Arduino.h>
// #include <LoRa.h>
// #include "Protocol.h"

// void setup()
// {
//     Serial.begin(115200);

//     LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
//     if (!LoRa.begin(433E6))
//     {
//         Serial.println("LoRa init failed.");
//         while (1)
//             ;
//     }
//     Serial.println("LoRa init succeeded.");
//     // LoRa.setSyncWord(0x12);
//     LoRa.setTxPower(15);
//     LoRa.setSpreadingFactor(13);

//     xSemaphore = xSemaphoreCreateMutex();
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
//             userDevice.receive(newReceivedPayload);
//         }
//     }
//     vTaskDelay(10 / portTICK_PERIOD_MS);
// }

#define RED_LED 27
#define BLUE_LED 2

void blinkRED();
void blinkBLUE();


#include <Arduino.h>
#include <LoRa.h>
#include "Protocol.h"
#include "BLE.h"

UserDevicePayload userDevice66(66);

UserDevicePayload userDevice77(77);

InterDevicePayload interDevice12(12, 3);
InterDevicePayload interDevice05(5, 2);

// BaseDevicePayload baseDevice0(0);
void sendPayload(const std::vector<int8_t> &payload)
{
    LoRa.beginPacket();
    for (int8_t byte : payload)
    {
        LoRa.write(byte);
    }
    LoRa.endPacket();
}

#define LORA_SS 5   // LoRa SPI CS pin
#define LORA_RST 14 // LoRa reset pin
#define LORA_DIO0 26

void setup()
{

    Serial.begin(115200);
    pinMode(RED_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);

    digitalWrite(RED_LED, HIGH);
    digitalWrite(BLUE_LED, HIGH);

    Serial.println("LoRa Device Simulator Starting...");
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    LoRa.setTxPower(17);        // Set transmission power to 17 dBm
    LoRa.setSpreadingFactor(7); // Set spreading factor to
    if (!LoRa.begin(433E6))
    {
        Serial.println("LoRa init failed. Check connections.");
        while (1)
            ;
    }
    Serial.println("LoRa init succeeded.");
}

void loop()
{

    userDevice66.createPmsg(4);
    userDevice66.loraSend();
    blinkBLUE();
    delay(5000);

    userDevice77.createCmsg("Hello from userDevice77", 0);
    userDevice77.loraSend();
    std::string msg1 = userDevice77.getJsonPayload();
    Serial.println(msg1.c_str());
        blinkBLUE();

    delay(5000);

    userDevice66.createGps(37.7749, -122.4194, 0);
    std::string msg2 = userDevice66.getJsonPayload();
    Serial.println(msg2.c_str());
    userDevice66.loraSend();
    blinkBLUE();

    delay(5000);

    userDevice77.createGps(34.0522, -118.2437, 0);
    std::string msg3 = userDevice77.getJsonPayload();
    Serial.println(msg3.c_str());
    userDevice77.loraSend();
        blinkBLUE();

    delay(5000);

    interDevice05.createPmsg(2);
    std::string msg4 = interDevice05.getJsonPayload();
    Serial.println(msg4.c_str());
    interDevice05.loraSend();
        blinkRED();

    delay(5000);

    interDevice12.createPmsg(5);
    std::string msg5 = interDevice12.getJsonPayload();
    Serial.println(msg5.c_str());
    interDevice12.loraSend();
    blinkRED();
    delay(5000);

    Serial.println("----------------");
    Serial.println();
    Serial.println("Looping through all devices...");
    blinkBLUE();
    delay(500);
    blinkRED();
}

// void sendPayload(const std::vector<int8_t>& payload ) {
//     LoRa.beginPacket();
//     for (int8_t byte : payload) {
//         LoRa.write(byte);
//     }
//     LoRa.endPacket();

// }


void blinkRED()
{
    digitalWrite(RED_LED, HIGH);
    delay(100);
    digitalWrite(RED_LED, LOW);
}

void blinkBLUE()
{
    digitalWrite(BLUE_LED, HIGH);
    delay(100);
    digitalWrite(BLUE_LED, LOW);
}
