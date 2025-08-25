#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <LoRa.h>

#define MAX_ACKS 10

class Payload
{
protected: // List of messages
    int type;
    int did;
    int dLvL;
    struct myPayload
    {
        int8_t dir;               // Direction of the message
        int8_t uid;               // User ID
        int8_t fLvL;              // Forwarding Level
        int8_t retry;             // Retry count
        int32_t mid;              // Message ID
        std::vector<int8_t> data; // Payload data
        int8_t chk;               // Checksum
    } MyPayload;

    struct AckItem
    {
        int8_t uid;
        int32_t mid;
    };
    int ackCount = 0;
    AckItem ackbucket[MAX_ACKS];
    int32_t currentMessageId = 1023;
    int counter = 0; // Current message ID

public:
    void clearPayload();
    int8_t calChecksumFromVector(const std::vector<int8_t> &payloadVector);
    int8_t calChecksumFromPayload();
    void setChecksum();
    bool verifyChecksum(const std::vector<int8_t> &payloadVector);

    void addAck(const std::vector<int8_t> &payloadVector);
    bool conformAck(const std::vector<int8_t> &payloadVector);
    void printAckBucket();

    void forwardPayload();

    void loraSend(); // sending current payload in MyPayload
    // init
    Payload(int type, int did, int dLvL);
    void printDeviceInfo();
    // set and get payloads
    bool setPayload(const std::vector<int8_t> &payloadVector);
    std::vector<int8_t> getPayload();
    void printPayload();
    std::string getJsonPayload();
    std::string getMsg(const std::vector<int8_t> &payloadVector);
    int8_t getDid();

    std::vector<std::string> getPredefinedMessagesForUser();
    std::vector<std::string> getPredefinedMessagesForBase();
};

class UserDevicePayload : public Payload
{
public:
    // u2b - 1
    UserDevicePayload(int did) : Payload(2, did, 127) {};
    struct Coordinate
    {
        float latitude;
        float longitude;
    };
    std::vector<Coordinate> gpsbucket;
    std::vector<std::string> gpsstringbucket; // Inbox for user device messages         // GPS data bucket
    std::vector<std::string> inboxbucket;     // Inbox for user device messages
    std::vector<std::string> sentboxbucket;   // Outbox for user device messages

    void createPmsg(int8_t pmsgid, int8_t attempts = 0);
    void createCmsg(std::string cmsg, int8_t attempts = 0);
    void createGps(float latitude, float longitude, int8_t attempts = 0);

    bool verifyRelation(const std::vector<int8_t> &payloadVector);
    // receive payload from base device only pmsg and cmsg
    bool receive(const std::vector<int8_t> &payloadVector);
    void setPayloadForward(std::vector<int8_t> payloadVector);

    std::vector<std::string> *getInboxBucket();
    std::vector<std::string> *getSentboxBucket();
    std::vector<Coordinate> *getGpsBucket();
    std::vector<std::string> *getGpsStringBucket();
    void setInboxNew(const std::string &message);
    void setSentboxNew(const std::string &message);
    void setGpsNew(const float &latitude, const float &longitude);
    void printStorage();
};

class InterDevicePayload : public Payload
{
public:
    InterDevicePayload(int did, int dLvL) : Payload(1, did, dLvL) {};
    bool verifyRelation(std::vector<int8_t> payloadVector);
    void createPmsg(int8_t pmsgid, int8_t attempts = 0);
    bool receive(std::vector<int8_t> payloadVector);
    void setPayloadForward(std::vector<int8_t> payloadVector);

private:
};

class BaseDevicePayload : public Payload
{
public:
    BaseDevicePayload(int did) : Payload(0, did, 0) {};
    void createPmsg(int8_t pmsgid, int8_t newUid, int8_t attempts = 0);
    void createCmsg(std::string cmsg, int8_t newUid, int8_t attempts = 0);
    void createGps(float latitude, float longitude, int8_t newUid, int8_t attempts = 0);
    int8_t getUid(const std::vector<int8_t> &payloadVector);

    bool receive(const std::vector<int8_t> &payloadVector);
    bool verifyRelation(std::vector<int8_t> payloadVector);

};

#endif

/*
    device types:
    0 - base Device , ids - 0 , LvL = 0
    1-inter Device ,ids 1-26 , LvL = 1-26
    2-user Device , ids 27-127, LvL = 127

    b2u - 0
    u2b - 1

    data types:
    pmsg - 0
    cmsg - 1
    gps - 2

*/