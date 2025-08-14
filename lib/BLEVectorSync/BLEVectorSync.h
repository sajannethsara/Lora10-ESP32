#pragma once
#include <NimBLEDevice.h>
#include <vector>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/**
 * BLEVectorSync
 *  - Exposes an existing std::vector<std::string> over BLE with 4 characteristics:
 *    size (READ, uint32 LE), index (WRITE, uint32 LE), data (READ, bytes), notify (NOTIFY, uint32 LE)
 *  - Thread-safe: all BLE reads and update checks lock a FreeRTOS mutex.
 *  - Your app must also lock this mutex when modifying the underlying vector.
 *
 * NOTE: The vector is passed as a const reference here, so the library won't modify it.
 *       Your app can still modify the original non-const vector instance.
 */
class BLEVectorSync {
public:
    BLEVectorSync(const char* name, const std::vector<std::string>& refVector);
    ~BLEVectorSync();

    // Create the 4 characteristics for this vector on a given service.
    void createCharacteristics(
        NimBLEService* svc,
        const char* sizeUUID,
        const char* indexUUID,
        const char* dataUUID,
        const char* notifyUUID
    );

    // Call periodically (e.g., in a FreeRTOS task) to emit NOTIFY when vector grows.
    void checkForUpdates();

    // Expose the mutex so app code can lock before pushing into the vector.
    SemaphoreHandle_t getMutex() const { return vecMutex; }

    // Optional: set maximum bytes returned by dataChar (0 = unlimited; default 0)
    void setMaxReadBytes(size_t n) { maxReadBytes = n; }

    const std::string name;
private:
    const std::vector<std::string>& vec;  // reference to your app's vector
    size_t lastSize = 0;
    uint32_t selectedIndex = 0;

    // optional limit for read size (safety for very long entries)
    size_t maxReadBytes = 0;

    // Mutex guarding reads & size tracking
    SemaphoreHandle_t vecMutex = nullptr;

    NimBLECharacteristic* sizeChar   = nullptr; // READ  : uint32 LE vector size
    NimBLECharacteristic* indexChar  = nullptr; // WRITE : uint32 LE index
    NimBLECharacteristic* dataChar   = nullptr; // READ  : string bytes at selectedIndex
    NimBLECharacteristic* notifyChar = nullptr; // NOTIFY: uint32 LE new index

    static std::string u32le(uint32_t v);
    static uint32_t leu32(const std::string& s);

    friend class _BVS_SizeCB;
    friend class _BVS_DataCB;
    friend class _BVS_IndexCB;
};
