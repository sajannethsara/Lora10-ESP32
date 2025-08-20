#include "BLEVectorSyncServer.h"
#include <cstdio>

BLEVectorSyncServer::BLEVectorSyncServer(const char *dn) {
    this->deviceName = dn;
}

void BLEVectorSyncServer::addVector(BLEVectorSync *v)
{
    vectors.push_back(v);
}

// Create stable UUIDs for each vector and each of its 4 characteristics.
// Format: "12345678-1234-5678-1234-56789abcVVTT"
//  - VV = vector index (2 hex digits, 00..FE)
//  - TT = type code (01 size, 02 index, 03 data, 04 notify)

// void BLEVectorSyncServer::makeVectorUUIDs(size_t i,
//                                           char *sizeUUID,
//                                           char *indexUUID,
//                                           char *dataUUID,
//                                           char *notifyUUID)
// {
//     const unsigned vecHex = static_cast<unsigned>(i & 0xFF);
//     // 36 chars + null -> buffer must be >= 37
//     std::snprintf(sizeUUID, 37, "12345678-1234-5678-1234-56789abc%02x01", vecHex);
//     std::snprintf(indexUUID, 37, "12345678-1234-5678-1234-56789abc%02x02", vecHex);
//     std::snprintf(dataUUID, 37, "12345678-1234-5678-1234-56789abc%02x03", vecHex);
//     std::snprintf(notifyUUID, 37, "12345678-1234-5678-1234-56789abc%02x04", vecHex);
// }
void BLEVectorSyncServer::makeVectorUUIDs(size_t i, char *sizeUUID, char *indexUUID, char *dataUUID, char *notifyUUID)
{
    const unsigned vecHex = static_cast<unsigned>(i & 0xFF);
    std::snprintf(sizeUUID, 37, "12345678-1234-5678-1234-56789abc%02x01", vecHex);
    std::snprintf(indexUUID, 37, "12345678-1234-5678-1234-56789abc%02x02", vecHex);
    std::snprintf(dataUUID, 37, "12345678-1234-5678-1234-56789abc%02x03", vecHex);
    std::snprintf(notifyUUID, 37, "12345678-1234-5678-1234-56789abc%02x04", vecHex);
    ESP_LOGI("BLE", "Vector %u UUIDs: size=%s, index=%s, data=%s, notify=%s",
             i, sizeUUID, indexUUID, dataUUID, notifyUUID);
}

class MyServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        Serial.printf("Client address: %s\n", connInfo.getAddress().toString().c_str());

        pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 180);
    }

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        Serial.printf("Client disconnected - start advertising\n");
        NimBLEDevice::startAdvertising();
    }

    void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override {
        Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
    }



} myServerCallbacks;


bool BLEVectorSyncServer::begin(uint16_t mtu, esp_power_level_t txPower)
{
    NimBLEDevice::setDeviceName(deviceName.c_str());
    if (!NimBLEDevice::init(deviceName) && !NimBLEDevice::setDeviceName(deviceName.c_str()))
    {
        Serial.printf("Failed to initialize NimBLE with device name: %s\n", deviceName.c_str());
        return false;
    }else
    {
        Serial.printf("NimBLE initialized with device name: %s\n", deviceName.c_str());
    }

    if (!NimBLEDevice::setMTU(mtu))
    {
        Serial.printf("Failed to set MTU to %d\n", mtu);
        return false;
    }
    else
    {
        Serial.printf("Successfully set MTU to %d\n", mtu);
    }
    NimBLEDevice::setPower(txPower);
    Serial.printf("Power set to %d\n", txPower);

    auto *server = NimBLEDevice::createServer();
    if (server == nullptr)
    {
        Serial.println("Failed to create server");
        return false;
    }
    server->setCallbacks(new MyServerCallbacks());
    Serial.println("Server callbacks set");

    auto *service = server->createService(SERVICE_UUID);
    if (service == nullptr)
    {
        Serial.println("Failed to create service");
        return false;
    }
    Serial.printf("Created service: %s\n", SERVICE_UUID);

    for (size_t i = 0; i < vectors.size(); ++i)
    {
        char sizeUUID[37], indexUUID[37], dataUUID[37], notifyUUID[37];
        makeVectorUUIDs(i, sizeUUID, indexUUID, dataUUID, notifyUUID);
        Serial.printf("Adding vector %d: %s\n", i, vectors[i]->name.c_str());
        vectors[i]->createCharacteristics(service, sizeUUID, indexUUID, dataUUID, notifyUUID);
    }

    if (!service->start())
    {
        Serial.println("Failed to start service");
        return false;
    }
    Serial.println("Service started");

    auto *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(SERVICE_UUID);
    adv->setName(deviceName.c_str());
    adv->setScanResponseData(adv->getAdvertisementData());
    Serial.printf("Starting advertising with name %s and service UUID %s\n",
                  deviceName.c_str(), SERVICE_UUID);
    if (!adv->start())
    {
        Serial.println("Failed to start advertising");
        return false;
    }
    Serial.println("Advertising started");
    return true;
}