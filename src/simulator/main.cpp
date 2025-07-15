#include <Arduino.h>
#include <LoRa.h>
#include "Protocol.h"

UserDevicePayload userDevice66(66);

UserDevicePayload userDevice77(77);
// BaseDevicePayload baseDevice0(0);
void sendPayload(const std::vector<int8_t>& payload ) {
    LoRa.beginPacket();
    for (int8_t byte : payload) {
        LoRa.write(byte);
    }
    LoRa.endPacket();

}


#define LORA_SS 5            // LoRa SPI CS pin
#define LORA_RST 14          // LoRa reset pin
#define LORA_DIO0 26

void setup() {
    

    Serial.begin(115200);
    Serial.println("LoRa Device Simulator Starting...");
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    LoRa.setTxPower(17); // Set transmission power to 17 dBm
    LoRa.setSpreadingFactor(7); // Set spreading factor to 7
    if (!LoRa.begin(433E6)) { 
        Serial.println("LoRa init failed. Check connections.");
        while (1);
    }
    Serial.println("LoRa init succeeded.");
}

void loop() {

    userDevice66.createPmsg(4);
    std::vector<int8_t> payload66 = userDevice66.getPayload();
    std::string jsonPayload66 = userDevice66.getJsonPayload(payload66);
    Serial.println(jsonPayload66.c_str());

    // sendPayload(payload66);
    // Serial.println("Payload sent from userDevice66");
    delay(500);
    // Simulate sending a Cmsg from userDevice77
    userDevice77.createCmsg("Hello from userDevice77", 0);
    std::vector<int8_t> payload77 = userDevice77.getPayload();
    std::string jsonPayload77 = userDevice77.getJsonPayload(payload77);
    Serial.println(jsonPayload77.c_str());
delay(500);
    userDevice66.createGps(37.7749, -122.4194, 0);
    std::vector<int8_t> gpsPayload66 = userDevice66.getPayload();
    std::string jsonGpsPayload66 = userDevice66.getJsonPayload(gpsPayload66);
    Serial.println(jsonGpsPayload66.c_str());
    delay(500);
    userDevice77.createGps(34.0522, -118.2437, 0);
    std::vector<int8_t> gpsPayload77 = userDevice77.getPayload();
    std::string jsonGpsPayload77 = userDevice77.getJsonPayload(gpsPayload77);
    Serial.println(jsonGpsPayload77.c_str());
delay(500);
    // Serial.println();
    // sendPayload(payload77);
    // Serial.println(typedef(payload66));
    delay(500);
    Serial.println("Ignore This Message");
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