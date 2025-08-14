#include "BLEVectorSync.h"
class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        Serial.printf("%s : onRead(), value: %s\n",
                      pCharacteristic->getUUID().toString().c_str(),
                      pCharacteristic->getValue().c_str());
    }

    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        Serial.printf("%s : onWrite(), value: %s\n",
                      pCharacteristic->getUUID().toString().c_str(),
                      pCharacteristic->getValue().c_str());
    }
} chrCallbacks;

// ---- Callbacks ----
// class _BVS_SizeCB : public NimBLECharacteristicCallbacks
// {
//     BLEVectorSync *ctx;

//     explicit _BVS_SizeCB(BLEVectorSync *c) : ctx(c) {}

//     void onRead(NimBLECharacteristic *ch) override
//     {
//         xSemaphoreTake(ctx->vecMutex, portMAX_DELAY);
//         uint32_t size = static_cast<uint32_t>(ctx->vec.size());
//         Serial.printf("BLEVectorSync: Size read: %u\n", size);
//         ch->setValue(BLEVectorSync::u32le(size));
//         xSemaphoreGive(ctx->vecMutex);
//     }
// };

class _BVS_SizeCB : public NimBLECharacteristicCallbacks {
    BLEVectorSync *ctx;

public:
    explicit _BVS_SizeCB(BLEVectorSync *c) : ctx(c) {
        Serial.printf("BLE: SizeCB created for %s\n", c->name.c_str());
    }

    void onRead(NimBLECharacteristic *ch, NimBLEConnInfo& connInfo) override {
        Serial.printf("BLE: Size read for %s, MTU: %u\n", ctx->name.c_str(), connInfo.getMTU());
        Serial.printf("BLEVectorSync: Size read for %s: %u\n", ctx->name.c_str(), ctx->vec.size());
        xSemaphoreTake(ctx->vecMutex, portMAX_DELAY);
        uint32_t size = static_cast<uint32_t>(ctx->vec.size());
        std::string value = BLEVectorSync::u32le(size);
        Serial.printf("BLE: Setting size value for %s: %02x %02x %02x %02x\n",
                      ctx->name.c_str(), (uint8_t)value[0], (uint8_t)value[1],
                      (uint8_t)value[2], (uint8_t)value[3]);
        ch->setValue(value);
        xSemaphoreGive(ctx->vecMutex);
    }
};

// class _BVS_DataCB : public NimBLECharacteristicCallbacks
// {
//     BLEVectorSync *ctx;

// public:
//     explicit _BVS_DataCB(BLEVectorSync *c) : ctx(c) {}
//     void onRead(NimBLECharacteristic *ch)
//     {
//         xSemaphoreTake(ctx->vecMutex, portMAX_DELAY);
//         std::string out;
//         if (ctx->selectedIndex < ctx->vec.size())
//         {
//             const std::string &s = ctx->vec.at(ctx->selectedIndex);
//             if (ctx->maxReadBytes > 0 && s.size() > ctx->maxReadBytes)
//             {
//                 out.assign(s.data(), ctx->maxReadBytes);
//             }
//             else
//             {
//                 out = s;
//             }
//         } // else out stays empty
//         xSemaphoreGive(ctx->vecMutex);
//         ch->setValue(out);
//     }
// };

class _BVS_DataCB : public NimBLECharacteristicCallbacks {
    BLEVectorSync *ctx;

public:
    explicit _BVS_DataCB(BLEVectorSync *c) : ctx(c) {
        Serial.printf("BLE: DataCB created for %s\n", c->name.c_str());
    }

    void onRead(NimBLECharacteristic *ch, NimBLEConnInfo& connInfo) override {
        Serial.printf("BLE: Data read for %s, MTU: %u, selected index: %u\n",
                      ctx->name.c_str(), connInfo.getMTU(), ctx->selectedIndex);

        std::string out;
        xSemaphoreTake(ctx->vecMutex, portMAX_DELAY);
        if (ctx->selectedIndex < ctx->vec.size()) {
            const std::string &s = ctx->vec.at(ctx->selectedIndex);
            if (ctx->maxReadBytes > 0 && s.size() > ctx->maxReadBytes) {
                out.assign(s.data(), ctx->maxReadBytes);
            } else {
                out = s;
            }
            Serial.printf("BLE: Returning %zu bytes for %s (index %u)\n",
                          out.size(), ctx->name.c_str(), ctx->selectedIndex);
        } else {
            Serial.printf("BLE: Selected index %u out of range for %s\n",
                          ctx->selectedIndex, ctx->name.c_str());
        }
        xSemaphoreGive(ctx->vecMutex);

        ch->setValue(out);
    }
};

// class _BVS_IndexCB : public NimBLECharacteristicCallbacks
// {
//     BLEVectorSync *ctx;

// public:
//     explicit _BVS_IndexCB(BLEVectorSync *c) : ctx(c) {}
//     void onWrite(NimBLECharacteristic *ch)
//     {
//         const std::string v = ch->getValue();
//         ctx->selectedIndex = BLEVectorSync::leu32(v);
//     }
// };

class _BVS_IndexCB : public NimBLECharacteristicCallbacks {
    BLEVectorSync *ctx;

public:
    explicit _BVS_IndexCB(BLEVectorSync *c) : ctx(c) {
        Serial.printf("BLE: IndexCB created for %s\n", c->name.c_str());
    }

    void onWrite(NimBLECharacteristic *ch, NimBLEConnInfo& connInfo) override {
        const std::string v = ch->getValue();
        if (v.size() < sizeof(uint32_t)) {
            Serial.printf("BLE: Index write for %s has invalid length: %zu\n",
                          ctx->name.c_str(), v.size());
            return;
        }

        uint32_t newIndex = BLEVectorSync::leu32(v);
        Serial.printf("BLE: Index write for %s, MTU: %u, new index: %u\n",
                      ctx->name.c_str(), connInfo.getMTU(), newIndex);

        xSemaphoreTake(ctx->vecMutex, portMAX_DELAY);
        ctx->selectedIndex = newIndex;
        xSemaphoreGive(ctx->vecMutex);
    }
};

// ---- BLEVectorSync ----
BLEVectorSync::BLEVectorSync(const char *n, const std::vector<std::string> &rv)
    : name(n), vec(rv)
{
    vecMutex = xSemaphoreCreateMutex();
}

BLEVectorSync::~BLEVectorSync()
{
    if (vecMutex)
    {
        vSemaphoreDelete(vecMutex);
        vecMutex = nullptr;
    }
}

void BLEVectorSync::createCharacteristics(
    NimBLEService* svc,
    const char* sizeUUID,
    const char* indexUUID,
    const char* dataUUID,
    const char* notifyUUID)
{
    // --- Create characteristics with proper properties ---
    sizeChar = svc->createCharacteristic(
        sizeUUID,
        NIMBLE_PROPERTY::READ
    );
    indexChar = svc->createCharacteristic(
        indexUUID,
        NIMBLE_PROPERTY::WRITE
    );
    dataChar = svc->createCharacteristic(
        dataUUID,
        NIMBLE_PROPERTY::READ
    );
    notifyChar = svc->createCharacteristic(
        notifyUUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    // --- Add user-friendly descriptors ---
    NimBLEDescriptor* sizeDesc = new NimBLEDescriptor(
        "2901", NIMBLE_PROPERTY::READ, name.length() + 6, sizeChar
    );
    sizeDesc->setValue(name + " Size");
    sizeChar->addDescriptor(sizeDesc);

    NimBLEDescriptor* indexDesc = new NimBLEDescriptor(
        "2901", NIMBLE_PROPERTY::READ, name.length() + 7, indexChar
    );
    indexDesc->setValue(name + " Index");
    indexChar->addDescriptor(indexDesc);

    NimBLEDescriptor* dataDesc = new NimBLEDescriptor(
        "2901", NIMBLE_PROPERTY::READ, name.length() + 6, dataChar
    );
    dataDesc->setValue(name + " Data");
    dataChar->addDescriptor(dataDesc);

    NimBLEDescriptor* notifyDesc = new NimBLEDescriptor(
        "2901", NIMBLE_PROPERTY::READ, name.length() + 8, notifyChar
    );
    notifyDesc->setValue(name + " Notify");
    notifyChar->addDescriptor(notifyDesc);

    // --- Assign callbacks ---
    sizeChar->setCallbacks(new _BVS_SizeCB(this));
    indexChar->setCallbacks(new _BVS_IndexCB(this));
    dataChar->setCallbacks(new _BVS_DataCB(this));
    // notifyChar does not need callbacks, you call notify() manually

    // --- Initialize lastSize ---
    xSemaphoreTake(vecMutex, portMAX_DELAY);
    lastSize = vec.size();
    xSemaphoreGive(vecMutex);
}


void BLEVectorSync::checkForUpdates()
{
    xSemaphoreTake(vecMutex, portMAX_DELAY);
    const size_t cur = vec.size();
    const bool grew = (cur > lastSize);
    const uint32_t newIdx = grew ? static_cast<uint32_t>(cur - 1) : 0;
    if (grew && notifyChar)
    {
        notifyChar->setValue(u32le(newIdx));
        notifyChar->notify();
    }
    lastSize = cur;
    xSemaphoreGive(vecMutex);
}

std::string BLEVectorSync::u32le(uint32_t v)
{
    std::string s(4, '\0');
    s[0] = static_cast<char>(v & 0xFF);
    s[1] = static_cast<char>((v >> 8) & 0xFF);
    s[2] = static_cast<char>((v >> 16) & 0xFF);
    s[3] = static_cast<char>((v >> 24) & 0xFF);
    return s;
}

uint32_t BLEVectorSync::leu32(const std::string &s)
{
    if (s.size() < 4)
        return 0;
    return static_cast<uint32_t>(static_cast<uint8_t>(s[0])) | (static_cast<uint32_t>(static_cast<uint8_t>(s[1])) << 8) | (static_cast<uint32_t>(static_cast<uint8_t>(s[2])) << 16) | (static_cast<uint32_t>(static_cast<uint8_t>(s[3])) << 24);
}
