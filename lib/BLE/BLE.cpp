#include "BLE.h"

MyBLEConnection::MyBLEConnection()
{
    //Setup BLE connection
}

void MyBLEConnection::StartBLE()
{
    Serial.println("Starting BLE...");
}

bool MyBLEConnection::ServeInboxChunks(int fromIndex, int chunkSize, const std::vector<std::string> &inbox)
{
    if (fromIndex < 0 || fromIndex >= inbox.size() || chunkSize <= 0) {
        Serial.println("Invalid parameters for ServeInboxChunks");
        return false;
    }

    // Prepare JSON output
    JsonDocument doc;
    JsonArray array = doc.to<JsonArray>();

    for (int i = fromIndex; i < fromIndex + chunkSize && i < inbox.size(); i++) {
        array.add(inbox[i]);
    }

    std::string output;
    serializeJson(doc, output);
    Serial.println(output.c_str());
    return true; // Return the JSON string
}
