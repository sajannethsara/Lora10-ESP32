#include <Arduino.h>
#include "Protocol.h"
#include <LoRa.h>


#define LORA_SS 5            // LoRa SPI CS pin
#define LORA_RST 14          // LoRa reset pin
#define LORA_DIO0 26         // LoRa IRQ pin

UserDevicePayload userDevice(53); // Declare userDevice globally

void setup() {
    Serial.begin(9600);
    // UserDevicePayload userDevice(53);
    Serial.println("Hello, World!");
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6))
  { // Set frequency to 433 MHz
    Serial.println("LoRa init failed. Check connections.");
    while (1)
      ;
  }
  Serial.println("LoRa init succeeded.");
}

void loop() {
    userDevice.createPmsg(1); 
    std::vector<int8_t> payload = userDevice.getPayload();
    Serial.println("Payload created with Pmsg");
    if (LoRa.begin(433E6)) {
        Serial.println("LoRa initialized successfully");
        LoRa.beginPacket();
        for (int8_t byte : payload) {
            LoRa.write(byte);
        }
        LoRa.endPacket();
        Serial.println("Payload sent via LoRa");
    } else {
        Serial.println("Failed to initialize LoRa");
    }
}