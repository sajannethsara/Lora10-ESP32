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

#include <Arduino.h>
#include <LoRa.h>
#include "Protocol.h"

UserDevicePayload userDevice66(66);

UserDevicePayload userDevice77(77);
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
    delay(5000);

    // sendPayload(payload66);
    // Serial.println("Payload sent from userDevice66");
    // Simulate sending a Cmsg from userDevice77
    userDevice77.createCmsg("Hello from userDevice77", 0);
    userDevice77.loraSend();
    std::string msg1 = userDevice77.getJsonPayload();
    Serial.println(msg1.c_str());
    delay(5000);

    userDevice66.createGps(37.7749, -122.4194, 0);
    std::string msg2 = userDevice66.getJsonPayload();
    Serial.println(msg2.c_str());
    userDevice66.loraSend();

    delay(5000);


    userDevice77.createGps(34.0522, -118.2437, 0);
    std::string msg3 = userDevice77.getJsonPayload();
    Serial.println(msg3.c_str());
    userDevice77.loraSend();
    delay(5000);

    // Serial.println();
    // sendPayload(payload77);
    // Serial.println(typedef(payload66));

    Serial.println("----------------");
    Serial.println();
    delay(500);
}

// void sendPayload(const std::vector<int8_t>& payload ) {
//     LoRa.beginPacket();
//     for (int8_t byte : payload) {
//         LoRa.write(byte);
//     }
//     LoRa.endPacket();

// }