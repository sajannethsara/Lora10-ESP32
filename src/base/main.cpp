// #include <Arduino.h>
// #include <Wire.h>
// #include <LoRa.h>
// #include "Protocol.h"
// #include <ArduinoJson.h>
// #include <LiquidCrystal_I2C.h>

// void _LoRaListenTask(void *pvParameters);
// void _LoRaSendTask(void *pvParameters);
// void _SerialListenTask(void *pvParameters);
// // void _LCDTask(void *pvParameters);

// TaskHandle_t Task1;
// TaskHandle_t Task2;
// TaskHandle_t Task3;
// TaskHandle_t Task4;

// #define LCD_ADDRESS 0x27 // Replace with your I2C address (e.g., 0x27 or 0x3F)
// #define LCD_COLUMNS 16   // Set to 20 for 20x4 LCD
// #define LCD_ROWS 2       // Set to 4 for 20x4 LCD
// LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

// void displayText(String message, int row);
// void clearLCD();
// void scrollText(String message, int row, int delayTime);
// void setupLCD();

// #define BUZZER_PIN 4
// #define PWM_CHANNEL 0
// #define PWM_FREQ 2000
// #define PWM_RESOLUTION 8
// void setupBuzzer();
// void beep();
// void beep2();
// void beep3();
// void noTone();

// #define LORA_SS 5
// #define LORA_RST 14
// #define LORA_DIO0 26
// int RSSI = -50;

// BaseDevicePayload baseDevice(0);
// SemaphoreHandle_t xSemaphore;
// QueueHandle_t loraQueue;
// std::string mostRecentMsg;
// int8_t mostRecentUid;
// void setMostRecentMsg(std::string msg, int8_t uid);

// void setup()
// {
//     Serial.begin(115200);

//     setupBuzzer();

//     LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
//     LoRa.setTxPower(17);
//     LoRa.setSpreadingFactor(7);
//     if (!LoRa.begin(433E6))
//     {
//         Serial.println("LoRa init failed.");
//         while (1)
//             ;
//     }
//     Serial.println("LoRa init succeeded.");
//     // LoRa.setSy1ncWord(0x12);
//     beep3();
//     // Initialize LCD
//     setupLCD();
//     displayText("LORA 10 v1.0", 0);
//     displayText("Base Station Device", 1);
//     loraQueue = xQueueCreate(10, sizeof(std::vector<int8_t> *));
//     xSemaphore = xSemaphoreCreateMutex();

//     xTaskCreatePinnedToCore(_LoRaListenTask, "LoRaListenTask", 8192, NULL, 1, &Task1, 0);
//     xTaskCreatePinnedToCore(_SerialListenTask, "SerialListenTask", 4096, NULL, 2, &Task2, 1);
//     xTaskCreatePinnedToCore(_LoRaSendTask, "LoRaSendTask", 2048, NULL, 3, &Task3, 1);
//     // xTaskCreatePinnedToCore(_LCDTask, "LCDTask", 2048, NULL, 1, &Task4, 1);
// }

// void _LoRaListenTask(void *pvParameters)
// {

//     while (1)
//     {
//         int packetSize = LoRa.parsePacket();
//         if (packetSize)
//         {
//             Serial.println("[Received packet]");
//             std::vector<int8_t> newReceivedPayload;
//             std::string jsonPayload;
//             RSSI = LoRa.packetRssi();
//             while (LoRa.available())
//             {
//                 newReceivedPayload.push_back(LoRa.read());
//             }

//             bool valid = baseDevice.receive(newReceivedPayload);
//             baseDevice.setPayload(newReceivedPayload);
//             std::string msg = baseDevice.getMsg(newReceivedPayload);
//             int8_t uid = baseDevice.getUid(newReceivedPayload);
//             if (valid)
//             {
//                 setMostRecentMsg(msg, uid);
//                 jsonPayload = baseDevice.getJsonPayload();
//                 Serial.println("[Payload Received]");
//                 Serial.println(jsonPayload.c_str());
//                 beep3();
//                 clearLCD();
//                 displayText("From: " + String(uid), 0);
//                 scrollText(String(msg.c_str()), 1, 1000);
//                 // baseDevice.printAckBucket();
//             }
//             // handleReceivedPayload(receivedPayload);
//         }
//         vTaskDelay(10 / portTICK_PERIOD_MS); // Check for new LoRa messages at a regular interval
//     }
// }

// void _SerialListenTask(void *pvParameters)
// {
//     JsonDocument doc;

//     while (1)
//     {
//         if (Serial.available())
//         {
//             String input = Serial.readStringUntil('\n');
//             DeserializationError error = deserializeJson(doc, input.c_str());

//             Serial.print("Received from Base Station : ");
//             Serial.println(input.c_str());

//             if (!error)
//             {
//                 int userId = doc["userId"] | -1;

//                 if (doc["data"].is<int>())
//                 {
//                     Serial.println("PMSG");
//                     int numberData = doc["data"];
//                     baseDevice.createPmsg(numberData, userId, 0);
//                     std::vector<int8_t> *payloadVector = new std::vector<int8_t>(baseDevice.getPayload());
//                     std::string msg = baseDevice.getMsg(*payloadVector);
//                     int8_t uid = baseDevice.getUid(*payloadVector);
//                     xQueueSend(loraQueue, &payloadVector, portMAX_DELAY);
//                     beep2();
//                     clearLCD();
//                     displayText("Sending to : " + String(uid), 0);
//                     scrollText(String(msg.c_str()), 1, 1000);
//                     vTaskDelay(1000 / portTICK_PERIOD_MS);
//                     clearLCD();
//                 }
//                 else if (doc["data"].is<const char *>())
//                 {
//                     Serial.println("CMSG");
//                     std::string stringData = doc["data"].as<const char *>();
//                     baseDevice.createCmsg(stringData, userId, 0);
//                     std::vector<int8_t> *payloadVector = new std::vector<int8_t>(baseDevice.getPayload());
//                     std::string msg = baseDevice.getMsg(*payloadVector);
//                     int8_t uid = baseDevice.getUid(*payloadVector);
//                     xQueueSend(loraQueue, &payloadVector, portMAX_DELAY);
//                     beep2();
//                     clearLCD();
//                     displayText("Sending to : " + String(uid), 0);
//                     scrollText(String(msg.c_str()), 1, 1000);
//                     vTaskDelay(1000 / portTICK_PERIOD_MS);
//                     clearLCD();
//                 }
//                 else
//                 {
//                     Serial.println("Unknown data type!");
//                 }
//             }
//             else
//             {
//                 Serial.println("JSON Parse Error!");
//             }
//         }
//         vTaskDelay(50 / portTICK_PERIOD_MS);
//     }
// }

// void _LoRaSendTask(void *pvParameters)
// {
//     for (;;)
//     {
//         std::vector<int8_t> *receivedVec;
//         if (xQueueReceive(loraQueue, &receivedVec, portMAX_DELAY))
//         {
//             bool pass = baseDevice.setPayload(*receivedVec);
//             printf("Payload set: %s\n", pass ? "[true]" : "[false]");
//             baseDevice.loraSend();
//             beep2();
//             delete receivedVec; // free when done
//         }
//         vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed
//     }
// }

// void loop() {}

// // sample dataaa
// // {"userId": 77,"type": "pmsg","data": 12}
// // {"userId": 77,"type": "cmsg","data": "Hi man rajuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu"}

// // {"userId": 77,"type": "cmsg","data": "Hi man raju"}

// void setMostRecentMsg(std::string msg, int8_t uid)
// {
//     if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(100)))
//     {
//         mostRecentMsg = msg;
//         mostRecentUid = uid;
//         xSemaphoreGive(xSemaphore);
//     }
// }

// void setupBuzzer()
// {
//     pinMode(BUZZER_PIN, OUTPUT);
//     ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
//     ledcAttachPin(BUZZER_PIN, PWM_CHANNEL);
//     noTone();
// }

// void beep()
// {
// #ifdef ACTIVE_BUZZER
//     digitalWrite(BUZZER_PIN, HIGH);
//     delay(100);
//     digitalWrite(BUZZER_PIN, LOW);
// #else
//     ledcWriteTone(PWM_CHANNEL, 1000);
//     delay(100);
//     noTone();
// #endif
// }

// void beep2()
// {
//     beep();
//     delay(100);
//     beep();
// }

// void beep3()
// {
// #ifdef ACTIVE_BUZZER
//     digitalWrite(BUZZER_PIN, HIGH);
//     delay(500);
//     digitalWrite(BUZZER_PIN, LOW);
//     delay(100);
//     digitalWrite(BUZZER_PIN, HIGH);
//     delay(500);
//     digitalWrite(BUZZER_PIN, LOW);
// #else
//     ledcWriteTone(PWM_CHANNEL, 500);
//     delay(500);
//     noTone();
//     delay(100);
//     ledcWriteTone(PWM_CHANNEL, 500);
//     delay(500);
//     noTone();
// #endif
// }

// void noTone()
// {
//     ledcWrite(PWM_CHANNEL, 0);
// }

// // Function to display a message on a specific row
// void displayText(String message, int row)
// {
//     if (row >= 0 && row < LCD_ROWS)
//     {
//         lcd.setCursor(0, row); // Set cursor to start of specified row
//         lcd.print(message);
//         // Clear remaining characters in the row to avoid leftover text
//         for (int i = message.length(); i < LCD_COLUMNS; i++)
//         {
//             lcd.print(" ");
//         }
//     }
// }

// // Function to clear the LCD
// void clearLCD()
// {
//     lcd.clear();
// }

// // Function to display a scrolling message on a specific row
// void scrollText(String message, int row, int delayTime)
// {
//     if (row >= 0 && row < LCD_ROWS)
//     {
//         // Pad message with spaces for smooth scrolling
//         String paddedMessage = message + " ";
//         for (int i = 0; i <= paddedMessage.length() - LCD_COLUMNS; i++)
//         {
//             lcd.setCursor(0, row);
//             lcd.print(paddedMessage.substring(i, i + LCD_COLUMNS));
//             vTaskDelay(pdMS_TO_TICKS(delayTime)); // Delay for scrolling effect
//         }
//     }
// }

// void setupLCD()
// {
//     // Initialize I2C with default pins (GPIO 21 SDA, GPIO 22 SCL)
//     // For alternate pins (e.g., TTGO T-Call), use: Wire.begin(18, 19);
//     Wire.begin();

//     // Initialize LCD
//     lcd.init();
//     lcd.backlight(); // Turn on backlight
//     lcd.clear();     // Clear the display
// }

#include <Arduino.h>
#include <Wire.h>
#include <LoRa.h>
#include "Protocol.h"
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>

// Task function declarations
void _LoRaListenTask(void *pvParameters);
void _LoRaSendTask(void *pvParameters);
void _SerialListenTask(void *pvParameters);

// Task handles
TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;

// LCD configuration
#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

// LCD function declarations
void displayText(String message, int row);
void clearLCD();
void scrollText(String message, int row, int delayTime);
void setupLCD();

// Buzzer configuration
#define BUZZER_PIN 4
#define PWM_CHANNEL 0
#define PWM_FREQ 2000
#define PWM_RESOLUTION 8
void setupBuzzer();
void beep();
void beep2();
void beep3();
void noTone();

// LoRa configuration
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 26
int RSSI = -50;
void onReceive(int packetSize);

// Payload structure for queue
struct PayloadData
{
    uint8_t data[256]; // Fixed size buffer
    size_t length;
    int8_t userId;
    bool isNumeric;
    union
    {
        int numericValue;
        char stringValue[200];
    } payload;
};

// Global variables
BaseDevicePayload baseDevice(0);
SemaphoreHandle_t xSemaphore;
QueueHandle_t loraQueue;
QueueHandle_t loraQueue2;
std::string mostRecentMsg;
int8_t mostRecentUid;

// System health monitoring
unsigned long lastSerialActivity = 0;
unsigned long lastLoRaActivity = 0;
bool systemHealthy = true;

void setMostRecentMsg(std::string msg, int8_t uid);
void clearSerialBuffer();
void systemHealthCheck();

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(1000); // Set timeout for Serial operations

    // Clear any existing serial data
    clearSerialBuffer();

    Serial.println("=== Base Device Starting ===");

    // Setup buzzer first
    setupBuzzer();

    // Initialize LCD early for status display
    setupLCD();
    displayText("Initializing...", 0);
    displayText("Please wait...", 1);

    // Initialize LoRa
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    // LoRa.setTxPower(17);
    // LoRa.setSpreadingFactor(13);

    if (!LoRa.begin(433E6))
    {
        Serial.println("ERROR: LoRa init failed!");
        displayText("LoRa FAILED!", 0);
        displayText("Check wiring", 1);
        while (1)
        {
            beep();
            delay(1000);
        }
    }
    LoRa.setFrequency(433E6);        // or 868E6 / 915E6 depending on module & region
    LoRa.setSpreadingFactor(13);     // 6–12 (12 = max range, min speed)
    LoRa.setSignalBandwidth(62.5E3); // 7.8kHz–500kHz (62.5kHz = good compromise)
    // LoRa.setCodingRate4(8);       // 4/5–4/8 (4/8 = strongest error correction)
    LoRa.enableCrc();                // ensure CRC check

    Serial.println("LoRa Config---");
    Serial.print("  Frequency: "); Serial.println(433E6);
    Serial.print("  TxPower: "); Serial.println(20);
    Serial.print("  SpreadingFactor: "); Serial.println(13);
    Serial.print("  SignalBandwidth: "); Serial.println(62.5E3);
    Serial.print("  CodingRate4: "); Serial.println(8);
    Serial.println("  CRC: Enabled");

    LoRa.onReceive(onReceive); // register callback
    LoRa.receive();            // put in receive mode

    Serial.println("SUCCESS: LoRa initialized");
    beep3();

    // Display startup message
    clearLCD();
    displayText("LORA 10 v1.0", 0);
    displayText("Base Station", 1);
    delay(2000);

    // Create queue and semaphore with error checking
    loraQueue = xQueueCreate(5, sizeof(PayloadData));  // Use fixed-size structure
    loraQueue2 = xQueueCreate(2, sizeof(PayloadData)); // Create second queue

    if (loraQueue == NULL || loraQueue2 == NULL)
    {
        Serial.println("ERROR: Failed to create LoRa queues 1 or 2");
        while (1)
            delay(1000);
    }

    xSemaphore = xSemaphoreCreateMutex();
    if (xSemaphore == NULL)
    {
        Serial.println("ERROR: Failed to create semaphore");
        while (1)
            delay(1000);
    }

    // Create tasks with increased stack sizes and error checking
    BaseType_t result;

    result = xTaskCreatePinnedToCore(_LoRaListenTask, "LoRaListenTask",
                                     12288, NULL, 2, &Task1, 0);
    if (result != pdPASS)
    {
        Serial.println("ERROR: Failed to create LoRa Listen Task");
        while (1)
            delay(1000);
    }

    result = xTaskCreatePinnedToCore(_SerialListenTask, "SerialListenTask",
                                     8192, NULL, 3, &Task2, 1);
    if (result != pdPASS)
    {
        Serial.println("ERROR: Failed to create Serial Listen Task");
        while (1)
            delay(1000);
    }

    result = xTaskCreatePinnedToCore(_LoRaSendTask, "LoRaSendTask",
                                     4096, NULL, 1, &Task3, 1);
    if (result != pdPASS)
    {
        Serial.println("ERROR: Failed to create LoRa Send Task");
        while (1)
            delay(1000);
    }

    Serial.println("SUCCESS: All tasks created");
    clearLCD();
    displayText("System Ready", 0);
    displayText("Waiting...", 1);

    lastSerialActivity = millis();
    lastLoRaActivity = millis();
}

void _LoRaListenTask(void *pvParameters)
{
    Serial.println("LoRa Listen Task started");

    while (1)
    {
        PayloadData receivedPayload;
        if (xQueueReceive(loraQueue2, &receivedPayload, portMAX_DELAY) == pdTRUE)
        {
            lastLoRaActivity = millis();
            RSSI = LoRa.packetRssi();
            int snr = LoRa.packetSnr();
            Serial.printf("[LoRa] RSSI: %d, SNR: %d\n", RSSI, snr);

            std::vector<int8_t> newReceivedPayload;
            newReceivedPayload.reserve(receivedPayload.length);

            for (size_t i = 0; i < receivedPayload.length; i++)
            {
                newReceivedPayload.push_back(receivedPayload.data[i]);
            }

            bool valid = baseDevice.receive(newReceivedPayload);
            if (valid)
            {
                baseDevice.setPayload(newReceivedPayload);
                std::string msg = baseDevice.getMsg(newReceivedPayload);
                int8_t uid = baseDevice.getUid(newReceivedPayload);

                setMostRecentMsg(msg, uid);

                Serial.println("[LoRa] Valid payload received:");
                Serial.println(baseDevice.getJsonPayload().c_str());

                beep3();
                clearLCD();
                displayText("From: " + String(uid), 0);

                String msgStr = String(msg.c_str());
                displayText(msgStr.substring(0, LCD_COLUMNS), 1);
            }
            else
            {
                Serial.println("[LoRa] Invalid payload received");
            }
        }
    }
}

void _SerialListenTask(void *pvParameters)
{
    Serial.println("Serial Listen Task started");

    // Allocate JSON document on the heap with proper size
    const size_t JSON_BUFFER_SIZE = 512;
    DynamicJsonDocument *doc = new DynamicJsonDocument(JSON_BUFFER_SIZE);

    if (!doc)
    {
        Serial.println("[Serial] ERROR: Failed to allocate JSON document");
        vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        try
        {
            if (Serial.available())
            {
                lastSerialActivity = millis();

                // Read input with timeout
                String input = Serial.readStringUntil('\n');
                input.trim(); // Remove whitespace

                // Skip empty inputs
                if (input.length() == 0)
                {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }

                Serial.printf("[Serial] Received: %s (length: %d)\n",
                              input.c_str(), input.length());

                // Clear the JSON document
                doc->clear();

                // Parse JSON
                DeserializationError error = deserializeJson(*doc, input);

                if (error)
                {
                    Serial.printf("[Serial] JSON Parse Error: %s\n", error.c_str());
                    clearSerialBuffer(); // Clear any remaining data
                    vTaskDelay(pdMS_TO_TICKS(50));
                    continue;
                }

                // Extract userId
                int userId = (*doc)["userId"] | -1;
                if (userId == -1)
                {
                    Serial.println("[Serial] Error: userId missing or invalid");
                    vTaskDelay(pdMS_TO_TICKS(50));
                    continue;
                }

                // Create payload data structure
                PayloadData payloadData;
                memset(&payloadData, 0, sizeof(PayloadData));
                payloadData.userId = userId;

                // Process data based on type
                bool success = false;

                if ((*doc)["data"].is<int>())
                {
                    Serial.println("[Serial] Processing PMSG");
                    int numberData = (*doc)["data"];
                    payloadData.isNumeric = true;
                    payloadData.payload.numericValue = numberData;

                    // Create message using a temporary BaseDevicePayload instance
                    BaseDevicePayload tempDevice(0);
                    tempDevice.createPmsg(numberData, userId, 0);
                    std::vector<int8_t> tempPayload = tempDevice.getPayload();
                    if (tempPayload.size() <= sizeof(payloadData.data))
                    {
                        memcpy(payloadData.data, tempPayload.data(), tempPayload.size());
                        payloadData.length = tempPayload.size();
                        success = true;
                    }
                }
                else if ((*doc)["data"].is<const char *>())
                {
                    Serial.println("[Serial] Processing CMSG");
                    const char *stringData = (*doc)["data"];

                    if (strlen(stringData) < sizeof(payloadData.payload.stringValue))
                    {
                        payloadData.isNumeric = false;
                        strncpy(payloadData.payload.stringValue, stringData,
                                sizeof(payloadData.payload.stringValue) - 1);
                        payloadData.payload.stringValue[sizeof(payloadData.payload.stringValue) - 1] = '\0';

                        // Create message using a temporary BaseDevicePayload instance
                        BaseDevicePayload tempDevice(0);
                        std::string stdStringData(stringData);
                        tempDevice.createCmsg(stdStringData, userId, 0);
                        std::vector<int8_t> tempPayload = tempDevice.getPayload();
                        if (tempPayload.size() <= sizeof(payloadData.data))
                        {
                            memcpy(payloadData.data, tempPayload.data(), tempPayload.size());
                            payloadData.length = tempPayload.size();
                            success = true;
                        }
                    }
                    else
                    {
                        Serial.println("[Serial] Error: String data too long");
                    }
                }
                else
                {
                    Serial.println("[Serial] Error: Unknown data type");
                    vTaskDelay(pdMS_TO_TICKS(50));
                    continue;
                }

                if (success && payloadData.length > 0)
                {
                    // Send to LoRa queue
                    if (xQueueSend(loraQueue, &payloadData, pdMS_TO_TICKS(1000)) == pdTRUE)
                    {
                        Serial.printf("[Serial] Queued message for UID %d\n", userId);

                        beep2();
                        clearLCD();
                        displayText("Sending to: " + String(userId), 0);

                        if (payloadData.isNumeric)
                        {
                            displayText("Num: " + String(payloadData.payload.numericValue), 1);
                        }
                        else
                        {
                            String msgStr = String(payloadData.payload.stringValue);
                            if (msgStr.length() <= LCD_COLUMNS)
                            {
                                displayText(msgStr, 1);
                            }
                            else
                            {
                                displayText(msgStr.substring(0, LCD_COLUMNS), 1);
                            }
                        }

                        vTaskDelay(pdMS_TO_TICKS(2000)); // Show message for 2 seconds
                        clearLCD();
                        displayText("Ready for input", 0);
                        displayText("", 1);
                    }
                    else
                    {
                        Serial.println("[Serial] Error: Failed to queue message");
                    }
                }
                else
                {
                    Serial.println("[Serial] Error: Failed to create payload");
                }
            }
        }
        catch (...)
        {
            Serial.println("[Serial] Exception in Serial Listen Task");
            clearSerialBuffer();
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Faster polling for better responsiveness
    }

    // Cleanup (this will never be reached, but good practice)
    delete doc;
}

void _LoRaSendTask(void *pvParameters)
{
    Serial.println("LoRa Send Task started");

    while (1)
    {
        try
        {
            PayloadData receivedPayload;

            // Wait for data in queue
            if (xQueueReceive(loraQueue, &receivedPayload, portMAX_DELAY) == pdTRUE)
            {
                Serial.println("[LoRa] Processing queued message");

                if (receivedPayload.length > 0 && receivedPayload.length <= sizeof(receivedPayload.data))
                {
                    // Create vector from fixed buffer
                    std::vector<int8_t> payloadVector;
                    payloadVector.reserve(receivedPayload.length);

                    for (size_t i = 0; i < receivedPayload.length; i++)
                    {
                        payloadVector.push_back(static_cast<int8_t>(receivedPayload.data[i]));
                    }

                    // Use a local BaseDevicePayload instance for sending
                    BaseDevicePayload sendDevice(0);
                    bool success = sendDevice.setPayload(payloadVector);

                    Serial.printf("[LoRa] Payload set: %s\n", success ? "SUCCESS" : "FAILED");

                    if (success)
                    {
                        sendDevice.loraSend();
                        Serial.println("[LoRa] Message sent successfully");
                        beep();
                    }
                    else
                    {
                        Serial.println("[LoRa] Failed to set payload");
                    }

                    // Clear the vector to free memory immediately  {userId : 42, data : "Hello Basssssss"}
                    payloadVector.clear();
                    payloadVector.shrink_to_fit();
                }
                else
                {
                    Serial.printf("[LoRa] Invalid payload length: %d\n", receivedPayload.length);
                }
                LoRa.receive(); // Return to receive mode
            }
        }
        catch (...)
        {
            Serial.println("[LoRa] Exception in LoRa Send Task");
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void loop()
{
    // System health monitoring
    systemHealthCheck();

    // Watchdog feed (if using hardware watchdog)
    delay(1000);
}

void setMostRecentMsg(std::string msg, int8_t uid)
{
    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        mostRecentMsg = msg;
        mostRecentUid = uid;
        xSemaphoreGive(xSemaphore);
    }
    else
    {
        Serial.println("[System] Warning: Failed to acquire semaphore");
    }
}

void clearSerialBuffer()
{
    while (Serial.available())
    {
        Serial.read();
    }
}

void systemHealthCheck()
{
    unsigned long currentTime = millis();

    // Check if tasks are responsive (optional - can be expanded)
    static unsigned long lastHealthCheck = 0;

    if (currentTime - lastHealthCheck > 30000) // Every 30 seconds
    {
        Serial.printf("[Health] System uptime: %lu ms\n", currentTime);
        Serial.printf("[Health] Last Serial activity: %lu ms ago\n",
                      currentTime - lastSerialActivity);
        Serial.printf("[Health] Last LoRa activity: %lu ms ago\n",
                      currentTime - lastLoRaActivity);
        Serial.printf("[Health] Free heap: %u bytes\n", ESP.getFreeHeap());

        lastHealthCheck = currentTime;
    }
}

// Buzzer functions (optimized)
void setupBuzzer()
{
    pinMode(BUZZER_PIN, OUTPUT);
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(BUZZER_PIN, PWM_CHANNEL);
    noTone();
}

void beep()
{
    ledcWriteTone(PWM_CHANNEL, 1000);
    vTaskDelay(pdMS_TO_TICKS(100));
    noTone();
}

void beep2()
{
    beep();
    vTaskDelay(pdMS_TO_TICKS(100));
    beep();
}

void beep3()
{
    ledcWriteTone(PWM_CHANNEL, 500);
    vTaskDelay(pdMS_TO_TICKS(500));
    noTone();
    vTaskDelay(pdMS_TO_TICKS(100));
    ledcWriteTone(PWM_CHANNEL, 500);
    vTaskDelay(pdMS_TO_TICKS(500));
    noTone();
}

void noTone()
{
    ledcWrite(PWM_CHANNEL, 0);
}

// LCD functions (optimized)
void displayText(String message, int row)
{
    if (row >= 0 && row < LCD_ROWS)
    {
        lcd.setCursor(0, row);

        // Ensure message doesn't exceed LCD width
        if (message.length() > LCD_COLUMNS)
        {
            message = message.substring(0, LCD_COLUMNS);
        }

        lcd.print(message);

        // Clear remaining characters
        for (int i = message.length(); i < LCD_COLUMNS; i++)
        {
            lcd.print(" ");
        }
    }
}

void clearLCD()
{
    lcd.clear();
}

void scrollText(String message, int row, int delayTime)
{
    if (row >= 0 && row < LCD_ROWS && message.length() > LCD_COLUMNS)
    {
        String paddedMessage = message + "    "; // Add padding

        for (int i = 0; i <= paddedMessage.length() - LCD_COLUMNS; i++)
        {
            lcd.setCursor(0, row);
            lcd.print(paddedMessage.substring(i, i + LCD_COLUMNS));
            vTaskDelay(pdMS_TO_TICKS(delayTime));
        }
    }
}

void setupLCD()
{
    Wire.begin();
    lcd.init();
    lcd.backlight();
    lcd.clear();
}

void onReceive(int packetSize)
{
    if (packetSize <= 0)
        return;

    PayloadData payloadData;
    memset(&payloadData, 0, sizeof(PayloadData));
    payloadData.length = packetSize;

    int index = 0;
    while (LoRa.available() && index < sizeof(payloadData.data))
    {
        payloadData.data[index++] = LoRa.read();
    }

    // Non-blocking send to queue
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(loraQueue2, &payloadData, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR(); // allow immediate context switch if needed
    }
}