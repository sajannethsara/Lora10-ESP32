#include "Protocol.h"

const std::string pmsgListForUser[] = {
    "Reached checkpoint",       // 0
    "Started hike",             // 1
    "Taking a break",           // 2
    "Low battery",              // 3
    "Need help",                // 4
    "Lost, need directions",    // 5
    "Injured, need assistance", // 6
    "Returning to base",        // 7
    "Reached destination",      // 8
    "Wild animal sighted",      // 9
    "Rain started",             // 10
    "Too dark to proceed",      // 11
    "All good",                 // 12
    "Slow progress",            // 13
    "No GPS signal"             // 14
};

const std::string pmsgListForBase[] = {
    "Status update?",            // 0
    "Received your location",    // 1
    "Help is on the way",        // 2
    "Take shelter immediately",  // 3
    "Return to base now",        // 4
    "Keep moving to next point", // 5
    "Activate emergency beacon", // 6
    "Rescue team deployed",      // 7
    "Check your surroundings",   // 8
    "Repeat last message",       // 9
    "Battery level?",            // 10
    "No signal from you",        // 11
    "Weather warning ahead",     // 12
    "Rest if needed",            // 13
    "Base station shutting down" // 14
};

int32_t int8ToInt32(int8_t b0, int8_t b1, int8_t b2, int8_t b3)
{
    return (static_cast<int32_t>(b0) & 0xFF) |
           ((static_cast<int32_t>(b1) & 0xFF) << 8) |
           ((static_cast<int32_t>(b2) & 0xFF) << 16) |
           ((static_cast<int32_t>(b3) & 0xFF) << 24);
}
int8_t int32ToInt8(int32_t value, int index)
{
    if (index < 0 || index > 3)
    {
        throw std::out_of_range("Index must be between 0 and 3.");
    }
    return static_cast<int8_t>((value >> (index * 8)) & 0xFF);
}

float int8ToFloat(int8_t b0, int8_t b1, int8_t b2, int8_t b3)
{
    int32_t intValue = int8ToInt32(b0, b1, b2, b3);
    return *reinterpret_cast<float *>(&intValue); // Reinterpret the bits as a float
}
int8_t floatToInt8(float value, int index)
{
    if (index < 0 || index > 3)
    {
        throw std::out_of_range("Index must be between 0 and 3.");
    }
    int32_t intValue = *reinterpret_cast<int32_t *>(&value); // Reinterpret the float as an int32_t
    return int32ToInt8(intValue, index);                     // Convert to int8_t using the existing function
}

// Checksum functions
int8_t Payload::calChecksumFromVector(const std::vector<int8_t> &payloadVector)
{
    int8_t checksum = 0;
    for (size_t i = 0; i < payloadVector.size() - 1; ++i) // Exclude the last item (checksum itself)
    {
        checksum += payloadVector[i];
    }
    return checksum;
}
int8_t Payload::calChecksumFromPayload()
{
    int8_t checksum = 0;
    checksum += MyPayload.dir;
    checksum += MyPayload.uid;
    checksum += MyPayload.fLvL;
    checksum += MyPayload.retry;
    checksum += (MyPayload.mid >> 24) & 0xFF; // First byte of mid
    checksum += (MyPayload.mid >> 16) & 0xFF; // Second byte of mid
    checksum += (MyPayload.mid >> 8) & 0xFF;  // Third byte of mid
    checksum += MyPayload.mid & 0xFF;         // Fourth byte of mid

    for (const auto &byte : MyPayload.data)
    {
        checksum += byte;
    }

    return checksum;
}
bool Payload::verifyChecksum(const std::vector<int8_t> &payloadVector)
{
    return payloadVector.back() == calChecksumFromVector(payloadVector);
}
// bool Payload::conformAck(const std::vector<int8_t> &payloadVector)
// {
//     for (auto it = ackbucket.begin(); it != ackbucket.end(); ++it)
//     {
//         if (it->uid == payloadVector[0] && it->mid == int8ToInt32(payloadVector[4], payloadVector[5], payloadVector[6], payloadVector[7]))
//         {
//             ackbucket.erase(it);
//             return false;
//         }
//     }
//     return true;
// }

bool Payload::conformAck(const std::vector<int8_t> &payloadVector)
{
    if (payloadVector.size() < 8)
        return true; // invalid payload, ignore

    int8_t uid = payloadVector[0];
    int32_t mid = int8ToInt32(payloadVector[4], payloadVector[5], payloadVector[6], payloadVector[7]);

    for (int i = 0; i < ackCount; i++)
    {
        if (ackbucket[i].uid == uid && ackbucket[i].mid == mid)
        {
            // Remove by shifting left
            for (int j = i; j < ackCount - 1; j++)
                ackbucket[j] = ackbucket[j + 1];

            ackCount--;
            return false; // Found ACK
        }
    }
    return true; // No ACK match
}

void Payload::printAckBucket()
{
    Serial.println("---- ACK Bucket ----");
    if (ackCount == 0)
    {
        Serial.println("Empty");
        return;
    }

    for (int i = 0; i < ackCount; i++)
    {
        Serial.print("Index ");
        Serial.print(i);
        Serial.print(": UID=");
        Serial.print(ackbucket[i].uid);
        Serial.print(", MID=");
        Serial.println(ackbucket[i].mid);
    }
    Serial.println("--------------------");
}

void Payload::addAck(const std::vector<int8_t> &payloadVector)
{   
    int8_t uid = payloadVector[1];
    int32_t mid = int8ToInt32(payloadVector[4], payloadVector[5], payloadVector[6], payloadVector[7]);
    // Check duplicate
    for (int i = 0; i < ackCount; i++)
    {
        if (ackbucket[i].uid == uid && ackbucket[i].mid == mid)
            return; // Already exists, no need to add
    }

    if (ackCount < MAX_ACKS)
    {
        // Add to end
        ackbucket[ackCount++] = {uid, mid};
    }
    else
    {
        // Overflow handling: remove oldest (shift left)
        for (int i = 1; i < MAX_ACKS; i++)
            ackbucket[i - 1] = ackbucket[i];

        // Add new at end
        ackbucket[MAX_ACKS - 1] = {uid, mid};
    }
}

void Payload::forwardPayload()
{
    MyPayload.fLvL = dLvL;
    setChecksum();
    loraSend();
}
void Payload::loraSend()
{
    LoRa.beginPacket();
    for (int8_t byte : getPayload())
    {
        LoRa.write(byte);
    }
    LoRa.endPacket();
    Serial.println("[Payload Sent]");
}
void Payload::setChecksum()
{
    MyPayload.chk = calChecksumFromPayload();
}

Payload::Payload(int input_type, int input_did, int input_dLvL)
{
    type = input_type;
    did = input_did;   // Correctly assign to the member variable did
    dLvL = input_dLvL; // Correctly assign to the member variable dLvL
}

void Payload::printDeviceInfo()
{
    std::cout << "---" << std::endl;
    std::cout << "Device ID: " << static_cast<int>(did) << std::endl;
    std::cout << "Device Level: " << static_cast<int>(dLvL) << std::endl;
    switch (type)
    {
    case 0:
        std::cout << "Device Type: 0:Base Device" << std::endl;
        break;
    case 1:
        std::cout << "Device Type: 1:Inter Device" << std::endl;
        break;
    case 2:
        std::cout << "Device Type: 2:User Device" << std::endl;
        break;
    default:
        break;
    }
    std::cout << "---" << std::endl;
}

bool Payload::setPayload(const std::vector<int8_t> &payloadVector)
{
    if (verifyChecksum(payloadVector))
    {
        clearPayload();
        MyPayload.dir = payloadVector[0];
        MyPayload.uid = payloadVector[1];
        MyPayload.fLvL = payloadVector[2];
        MyPayload.retry = payloadVector[3];
        MyPayload.mid = int8ToInt32(payloadVector[4], payloadVector[5], payloadVector[6], payloadVector[7]);
        MyPayload.data = std::vector<int8_t>(payloadVector.begin() + 8, payloadVector.end() - 1);
        setChecksum(); // Set the checksum after populating MyPayload
        return true;   // Indicate success
    }
    else
    {
        std::cerr << "Checksum verification failed. Payload not set." << std::endl;
        // clearPayload();
        return false; // Indicate failure
    } // Verify checksum before setting payload
}

void Payload::printPayload()
{
    std::cout << "Direction: " << static_cast<int>(MyPayload.dir) << std::endl;
    std::cout << "User ID: " << static_cast<int>(MyPayload.uid) << std::endl;
    std::cout << "Forwarding Level: " << static_cast<int>(MyPayload.fLvL) << std::endl;
    std::cout << "Retry Count: " << static_cast<int>(MyPayload.retry) << std::endl;
    std::cout << "Message ID: " << MyPayload.mid << std::endl;

    std::cout << "Payload Data: ";
    for (const auto &byte : MyPayload.data)
    {
        std::cout << static_cast<int>(byte) << " ";
    }
    std::cout << std::endl;

    std::cout << "Checksum: " << static_cast<int>(MyPayload.chk) << std::endl;
}

std::string Payload::getJsonPayload()
{
    // make data json object- these objects are diffrent by type
    std::ostringstream oss;

    // Start JSON object
    oss << "{";

    // Add common fields
    oss << "\"dir\": " << static_cast<int>(MyPayload.dir) << ", ";
    oss << "\"uid\": " << static_cast<int>(MyPayload.uid) << ", ";
    oss << "\"fLvL\": " << static_cast<int>(MyPayload.fLvL) << ", ";
    oss << "\"retry\": " << static_cast<int>(MyPayload.retry) << ", ";
    oss << "\"mid\": " << MyPayload.mid << ", ";

    // Add data field
    oss << "\"data\": {";
    switch (MyPayload.data[0])
    {
    case 0: // Pmsg
        oss << "\"type\": 0, ";
        oss << "\"pmsgid\": " << static_cast<int>(MyPayload.data[1]) << " ";
        break;
    case 1: // Cmsg
        oss << "\"type\": 1, ";
        oss << "\"cmsg\": \"" << std::string(MyPayload.data.begin() + 2, MyPayload.data.end()) << "\" ";
        break;
    case 2: // Gps
        oss << "\"type\": 2, ";
        oss << "\"lat\": " << int8ToFloat(MyPayload.data[1], MyPayload.data[2], MyPayload.data[3], MyPayload.data[4]) << ", ";
        oss << "\"lon\": " << int8ToFloat(MyPayload.data[5], MyPayload.data[6], MyPayload.data[7], MyPayload.data[8]) << " ";
        break;
    default:
        break;
    }
    oss << "}";

    // Add checksum
    oss << ", \"chk\": " << static_cast<int>(MyPayload.chk);

    // End JSON object
    oss << "}";

    return oss.str();
}

std::vector<int8_t> Payload::getPayload()
{
    std::vector<int8_t> payloadVector;
    payloadVector.push_back(MyPayload.dir);
    payloadVector.push_back(MyPayload.uid);
    payloadVector.push_back(MyPayload.fLvL);
    payloadVector.push_back(MyPayload.retry);
    payloadVector.push_back(int32ToInt8(MyPayload.mid, 0));
    payloadVector.push_back(int32ToInt8(MyPayload.mid, 1));
    payloadVector.push_back(int32ToInt8(MyPayload.mid, 2));
    payloadVector.push_back(int32ToInt8(MyPayload.mid, 3));
    payloadVector.insert(payloadVector.end(), MyPayload.data.begin(), MyPayload.data.end());
    payloadVector.push_back(MyPayload.chk);
    return payloadVector;
}

void Payload::clearPayload()
{
    MyPayload = {}; // Reset all fields to zero/empty
}

void UserDevicePayload::createPmsg(int8_t pmsgid, int8_t attempts)
{
    clearPayload();        // Clear any existing payload data
    MyPayload.dir = 1;     // u2b
    MyPayload.uid = did;   // Set User ID to device ID
    MyPayload.fLvL = dLvL; // Set Forwarding Level to device level
    MyPayload.mid = ++currentMessageId;
    MyPayload.retry = attempts; // Initialize retry count to 0
    MyPayload.data.clear();
    // Clear any existing data
    std::vector<int8_t> dataVector = {0, pmsgid};
    MyPayload.data = dataVector;
    setChecksum(); // Set the checksum after populating MyPayload
}

void UserDevicePayload::createCmsg(std::string cmsg, int8_t attempts)
{
    clearPayload();        // Clear any existing payload data
    MyPayload.dir = 1;     // u2b
    MyPayload.uid = did;   // Set User ID to device ID
    MyPayload.fLvL = dLvL; // Set Forwarding Level to device level
    MyPayload.mid = ++currentMessageId;
    MyPayload.retry = attempts; // Set retry count
    MyPayload.data.clear();     // Clear any existing data

    // Populate data vector with message type and size
    MyPayload.data.push_back(1);                                // Message type: 1 (Cmsg)
    MyPayload.data.push_back(static_cast<int8_t>(cmsg.size())); // Message size

    // Append message content as bytes
    MyPayload.data.insert(MyPayload.data.end(), cmsg.begin(), cmsg.end());

    setChecksum(); // Calculate and set checksum
}

void UserDevicePayload::createGps(float latitude, float longitude, int8_t attempts)
{
    clearPayload();        // Clear any existing payload data
    MyPayload.dir = 1;     // u2b
    MyPayload.uid = did;   // Set User ID to device ID
    MyPayload.fLvL = dLvL; // Set Forwarding Level to device level
    MyPayload.mid = ++currentMessageId;
    MyPayload.retry = attempts; // Set retry count
    MyPayload.data.clear();     // Clear any existing data

    // Populate data vector with message type and GPS coordinates
    MyPayload.data.push_back(2); // Message type: 2 (Gps)
    MyPayload.data.push_back(floatToInt8(latitude, 0));
    MyPayload.data.push_back(floatToInt8(latitude, 1));
    MyPayload.data.push_back(floatToInt8(latitude, 2));
    MyPayload.data.push_back(floatToInt8(latitude, 3));
    MyPayload.data.push_back(floatToInt8(longitude, 0));
    MyPayload.data.push_back(floatToInt8(longitude, 1));
    MyPayload.data.push_back(floatToInt8(longitude, 2));
    MyPayload.data.push_back(floatToInt8(longitude, 3));

    setChecksum(); // Calculate and set checksum
}

bool UserDevicePayload::verifyRelation(std::vector<int8_t> payloadVector)
{
    if (payloadVector[0] == 0 && payloadVector[1] == did)
    {
        return true;
    }
    else
    {
        return false;
    }
}
// device types:
// 0 - base Device , ids - 0 , LvL = 0
// 1-inter Device ,ids 1-26 , LvL = 1-26
// 2-user Device , ids 27-127, LvL = 127

// b2u - 0
// u2b - 1

bool InterDevicePayload::verifyRelation(std::vector<int8_t> payloadVector)
{
    int8_t msgLvL = payloadVector[2];
    if (payloadVector[0] == 0) // b2u
    {
        if (dLvL > msgLvL)
        {
            return true;
        }
        else
        {
            return false; // If the device level is not higher than the message level, do not accept
        }
    }
    else if (payloadVector[0] == 1) // u2b
    {
        if (dLvL < msgLvL)
        {
            return true;
        }
        else
        {
            return false; // If the device level is not lower than the message level, do not accept
        }
    }
    return false; // Default return value if none of the conditions are met
}

void UserDevicePayload::receive(std::vector<int8_t> payloadVector)
{ // receive payload from base device only pmsg and cmsg
    bool pass1 = Payload::verifyChecksum(payloadVector);
    bool pass2 = Payload::conformAck(payloadVector);
    bool pass3 = UserDevicePayload::verifyRelation(payloadVector);
    if (pass1 && !pass2 && pass3)
    {
        if (payloadVector[8] == 0) // pmsg
        {
            inboxbucket.push_back(pmsgListForBase[payloadVector[9]]);
        }
        else if (payloadVector[8] == 1) // cmsg
        {
            std::string message(payloadVector.begin() + 9, payloadVector.end() - 1); // Exclude checksum
            inboxbucket.push_back(message);
        }
        forwardPayload(); // Set the payload after processing
    }
}

bool BaseDevicePayload::receive(const std::vector<int8_t> &payloadVector)
{
    // Process the received payload
    bool pass1 = Payload::verifyChecksum(payloadVector);
    bool pass2 = Payload::conformAck(payloadVector);
    if (pass1 && pass2)
    {
        return setPayload(payloadVector);
        // if (p) return Payload::getJsonPayload(payloadVector);
    }
    else
    {
        return false; // Indicate failure
    }
}

bool InterDevicePayload::receive(std::vector<int8_t> payloadVector)
{
    // Process the received payload
    bool pass1 = Payload::verifyChecksum(payloadVector);
    bool pass2 = Payload::conformAck(payloadVector);
    bool pass3 = InterDevicePayload::verifyRelation(payloadVector);

    if (pass1 && !pass2 && pass3)
    {
        return setPayload(payloadVector);
        // if (p) return Payload::getJsonPayload(payloadVector);
    }
    else
    {
        return false; // Indicate failure
    }
}

void InterDevicePayload::setPayloadForward(std::vector<int8_t> payloadVector)
{
    setPayload(payloadVector);
    MyPayload.retry++;
    MyPayload.fLvL = dLvL; // Set Forwarding Level to device level
    setChecksum();         // Set the checksum after populating MyPayload
}

std::vector<std::string> UserDevicePayload::getInbox()
{
    return inboxbucket;
}

void UserDevicePayload::setInboxNew(const std::string &message)
{
    inboxbucket.push_back(message);
}

void UserDevicePayload::printInbox()
{
    Serial.println("---- Inbox ----");
    if (inboxbucket.empty())
    {
        Serial.println("Inbox is empty.");
        return;
    }

    for (size_t i = 0; i < inboxbucket.size(); ++i)
    {
        Serial.print("Message ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(inboxbucket[i].c_str());
    }
    Serial.println("----------------");
}

void BaseDevicePayload::createPmsg(int8_t pmsgid, int8_t newUid, int8_t attempts)
{
    clearPayload();        // Clear any existing payload data
    MyPayload.dir = 1;     // u2b
    MyPayload.uid = did;   // Set User ID to device ID
    MyPayload.fLvL = dLvL; // Set Forwarding Level to device level
    MyPayload.mid = ++currentMessageId;
    MyPayload.retry = attempts; // Initialize retry count to 0
    MyPayload.data.clear();
    // Clear any existing data
    std::vector<int8_t> dataVector = {0, pmsgid};
    MyPayload.data = dataVector;
    setChecksum(); // Set the checksum after populating MyPayload
}

void InterDevicePayload::createPmsg(int8_t pmsgid, int8_t attempts)
{
    clearPayload();        // Clear any existing payload data
    MyPayload.dir = 0;     // b2u
    MyPayload.uid = did;   // Set User ID to device ID
    MyPayload.fLvL = dLvL; // Set Forwarding Level to device level
    MyPayload.mid = ++currentMessageId;
    MyPayload.retry = attempts; // Initialize retry count to 0
    MyPayload.data.clear();
    // Clear any existing data
    std::vector<int8_t> dataVector = {0, pmsgid};
    MyPayload.data = dataVector;
    setChecksum(); // Set the checksum after populating MyPayload
}

void BaseDevicePayload::createCmsg(std::string cmsg, int8_t newUid, int8_t attempts)
{
    clearPayload();         // Clear any existing payload data
    MyPayload.dir = 0;      // b2u
    MyPayload.uid = newUid; // Set User ID to device ID
    MyPayload.fLvL = dLvL;  // Set Forwarding Level to device level
    MyPayload.mid = ++currentMessageId;
    MyPayload.retry = attempts; // Set retry count
    MyPayload.data.clear();     // Clear any existing data

    // Populate data vector with message type and size
    MyPayload.data.push_back(1);                                // Message type: 1 (Cmsg)
    MyPayload.data.push_back(static_cast<int8_t>(cmsg.size())); // Message size

    // Append message content as bytes
    MyPayload.data.insert(MyPayload.data.end(), cmsg.begin(), cmsg.end());

    setChecksum(); // Calculate and set checksum
}

void BaseDevicePayload::createGps(float latitude, float longitude, int8_t newUid, int8_t attempts)
{
    clearPayload();         // Clear any existing payload data
    MyPayload.dir = 0;      // b2u
    MyPayload.uid = newUid; // Set User ID to device ID
    MyPayload.fLvL = dLvL;  // Set Forwarding Level to device level
    MyPayload.mid = ++currentMessageId;
    MyPayload.retry = attempts; // Set retry count
    MyPayload.data.clear();     // Clear any existing data

    // Populate data vector with message type and GPS coordinates
    MyPayload.data.push_back(2); // Message type: 2 (Gps)
    MyPayload.data.push_back(floatToInt8(latitude, 0));
    MyPayload.data.push_back(floatToInt8(latitude, 1));
    MyPayload.data.push_back(floatToInt8(latitude, 2));
    MyPayload.data.push_back(floatToInt8(latitude, 3));
    MyPayload.data.push_back(floatToInt8(longitude, 0));
    MyPayload.data.push_back(floatToInt8(longitude, 1));
    MyPayload.data.push_back(floatToInt8(longitude, 2));
    MyPayload.data.push_back(floatToInt8(longitude, 3));

    setChecksum(); // Calculate and set checksum
}

void receive(std::vector<int8_t> payloadVector) {}