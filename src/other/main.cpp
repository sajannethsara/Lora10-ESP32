#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <U8g2lib.h>

// LoRa Pins
#define SS 5
#define RST 14
#define DIO0 2

// OLED display settings (example: SSD1306 128x64 I2C)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void setup() {
    Serial.begin(115200);

    // Initialize OLED
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 12, "LoRa Receiver Init");
    u8g2.sendBuffer();

    // Initialize LoRa
    LoRa.setPins(SS, RST, DIO0);
    if (!LoRa.begin(433E6)) {
        u8g2.clearBuffer();
        u8g2.drawStr(0, 12, "LoRa Failed!");
        u8g2.sendBuffer();
        while (true);
    }

    // Max Range Settings
    LoRa.setSpreadingFactor(12);
    LoRa.setSignalBandwidth(62.5E3);
    LoRa.setCodingRate4(8);
    LoRa.setTxPower(20);

    u8g2.clearBuffer();
    u8g2.drawStr(0, 12, "LoRa Receiver OK");
    u8g2.sendBuffer();
    delay(2000);
    u8g2.clearBuffer();
    u8g2.sendBuffer();

    Serial.println("LoRa Receiver Ready");
}

void loop() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        String received = "";
        while (LoRa.available()) {
            received += (char)LoRa.read();
        }

        int rssi = LoRa.packetRssi();

        // Debug in Serial
        Serial.println("====================");
        Serial.println("Received: " + received);
        Serial.print("RSSI: ");
        Serial.println(rssi);
        Serial.println("====================");

        // Display on OLED
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(0, 12, "Msg Received:");
        u8g2.drawStr(0, 28, received.c_str());
        char rssiStr[20];
        snprintf(rssiStr, sizeof(rssiStr), "RSSI: %d", rssi);
        u8g2.drawStr(0, 44, rssiStr);
        u8g2.sendBuffer();
    }
}
