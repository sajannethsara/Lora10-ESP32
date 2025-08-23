#pragma once
#include <NimBLEDevice.h>
#include <vector>
#include <string>
#include "BLEVectorSync.h"

/**
 * BLEVectorSyncServer
 *  - Initializes NimBLE, creates one service, and installs N vectors (each with 4 characteristics).
 *  - You still call checkForUpdates() on each BLEVectorSync from your task loop.
 */
class BLEVectorSyncServer {
public:
    explicit BLEVectorSyncServer(const char* deviceName);
    using DataCallback = std::function<void(const std::string&)>;
    void setDataCallback(DataCallback cb);
    // Add a vector before begin()
    void addVector(BLEVectorSync* v);

    // Initialize BLE, create service and characteristics, start advertising.
    bool begin(uint16_t mtu = 120, esp_power_level_t txPower = ESP_PWR_LVL_P9);

    std::string deviceName;
    DataCallback dataCallback;
private:
    std::vector<BLEVectorSync*> vectors;
    // A fixed base UUID for the service (you can change it if you want)
    static constexpr const char* SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0";

    // Deterministic per-vector characteristic UUIDs (Size/Index/Data/Notify)
    // Generates printable 36-char UUID strings into provided buffers.
    static void makeVectorUUIDs(size_t i,
                                char* sizeUUID,
                                char* indexUUID,
                                char* dataUUID,
                                char* notifyUUID);
};
