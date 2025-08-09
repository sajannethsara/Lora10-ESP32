#ifndef BLE_H
#define BLE_H

#include "Protocol.h"
#include <NimBLEDevice.h>
#include <ArduinoJson.h>
// types : inbox - 0 , sendbox - 1 , gps - 2

class MyBLEConnection
{
protected:
public:
    MyBLEConnection();

    void StartBLE();

    // bool FlagInboxBLE();
    // bool FlagSendboxBLE();
    // bool FlagGpsBLE();

    void HandleRequest(int type , int chunk, int size);

    bool ServeInboxChunks(int fromIndex, int chunkSize, const std::vector<std::string> &inbox);
    // bool ServeSendboxChunks(int chunk, int size, const std::vector<std::string> &sendbox);
    // bool ServeGpsChunks(int chunk, int size, const std::vector<std::string> &gps);
};

#endif
