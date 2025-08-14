#include <Arduino.h>
#include "BLEVectorSyncServer.h"

// Your app's vectors (non-const so you can push; library holds const& to them)
std::vector<std::string> inbox;
std::vector<std::string> sendbox;
std::vector<std::string> gps;

// Sync wrappers
BLEVectorSyncServer bleServer("ESP32_Sync_Device");
BLEVectorSync inboxSync("inbox", inbox);
BLEVectorSync sendboxSync("sendbox", sendbox);
BLEVectorSync gpsSync("gps", gps);

// Example: safely push into a vector using its mutex
static void pushInboxSafe(const std::string &s)
{
    xSemaphoreTake(inboxSync.getMutex(), portMAX_DELAY);
    inbox.push_back(s);
    xSemaphoreGive(inboxSync.getMutex());
}

static void pushSendboxSafe(const std::string &s)
{
    xSemaphoreTake(sendboxSync.getMutex(), portMAX_DELAY);
    sendbox.push_back(s);
    xSemaphoreGive(sendboxSync.getMutex());
}

static void pushGpsSafe(const std::string &s)
{
    xSemaphoreTake(gpsSync.getMutex(), portMAX_DELAY);
    gps.push_back(s);
    xSemaphoreGive(gpsSync.getMutex());
}

void setup()
{
    Serial.begin(115200);
    delay(100);

    // Optional: cap max read bytes (0 = unlimited, rely on MTU)
    inboxSync.setMaxReadBytes(240);

    // Register vectors
    bleServer.addVector(&inboxSync);
    bleServer.addVector(&sendboxSync);
    bleServer.addVector(&gpsSync);

    // Start BLE (MTU 247 ~ 244 bytes payload per GATT read)
    while (!bleServer.begin(120, ESP_PWR_LVL_P9))
    {
        Serial.println("Failed to start BLE server, retrying...");
        delay(1000);
    }
    Serial.println("[BLE] server started successfully");
    // Seed demo data (thread-safe)
    pushInboxSafe("1: hello");
    pushInboxSafe("3: how are you?");
    pushSendboxSafe("2: sent OK");
    pushGpsSafe("7.8731,80.7718");
}
int count = 0;
void loop()
{
    // Periodically check for vector growth and notify subscribers
    inboxSync.checkForUpdates();
    sendboxSync.checkForUpdates();
    gpsSync.checkForUpdates();

    // Demo: append new items every 10s
    static uint32_t last = 0;
    if (millis() - last > 10000)
    {
        last = millis();
        pushInboxSafe("1: hello");
        pushInboxSafe("3: how are you?" + std::to_string(count++));
        pushSendboxSafe("2: sent OK");
        pushGpsSafe("7.8731,80.7718");
        Serial.println("Appended demo items.");
    }

    delay(200);
}
