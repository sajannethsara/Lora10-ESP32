#include <Arduino.h>
#include <Wire.h>
#include <LoRa.h>
#include "Protocol.h"
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>

void _LoRaListenTask(void *pvParameters);
void _LoRaSendTask(void *pvParameters);
void _SerialListenTask(void *pvParameters);
void _LCDTask(void *pvParameters);

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;

#define LCD_ADDRESS 0x27 // Replace with your I2C address (e.g., 0x27 or 0x3F)
#define LCD_COLUMNS 16   // Set to 20 for 20x4 LCD
#define LCD_ROWS 2       // Set to 4 for 20x4 LCD
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

void displayText(String message, int row);
void clearLCD();
void scrollText(String message, int row, int delayTime);
void setupLCD();

#define BUZZER_PIN 4
#define PWM_CHANNEL 0
#define PWM_FREQ 2000
#define PWM_RESOLUTION 8
void setupBuzzer();
void beep();
void beep2();
void beep3();
void noTone();

#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 26
int RSSI = -50;

BaseDevicePayload baseDevice(0);
SemaphoreHandle_t xSemaphore;
QueueHandle_t loraQueue;
std::string mostRecentMsg;
int8_t mostRecentUid;
void setMostRecentMsg(std::string msg, int8_t uid);

void setup()
{
    Serial.begin(115200);

    setupBuzzer();

    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    LoRa.setTxPower(17);
    LoRa.setSpreadingFactor(7);
    if (!LoRa.begin(433E6))
    {
        Serial.println("LoRa init failed.");
        while (1)
            ;
    }
    Serial.println("LoRa init succeeded.");
    // LoRa.setSy1ncWord(0x12);
    beep3();
    // Initialize LCD
    setupLCD();
    displayText("Hello, World!", 0);
    displayText("Huiiiiiiiiiiii", 1);
    loraQueue = xQueueCreate(10, sizeof(std::vector<int8_t> *));
    xSemaphore = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(_LoRaListenTask, "LoRaListenTask", 8192, NULL, 1, &Task1, 0);
    xTaskCreatePinnedToCore(_SerialListenTask, "SerialListenTask", 4096, NULL, 2, &Task2, 1);
    xTaskCreatePinnedToCore(_LoRaSendTask, "LoRaSendTask", 2048, NULL, 3, &Task3, 1);
    xTaskCreatePinnedToCore(_LCDTask, "LCDTask", 2048, NULL, 1, &Task4, 1);
}

void _LoRaListenTask(void *pvParameters)
{

    while (1)
    {
        int packetSize = LoRa.parsePacket();
        if (packetSize)
        {
            Serial.println("[Received packet]");
            std::vector<int8_t> newReceivedPayload;
            std::string jsonPayload;
            RSSI = LoRa.packetRssi();
            while (LoRa.available())
            {
                newReceivedPayload.push_back(LoRa.read());
            }

            bool valid = baseDevice.receive(newReceivedPayload);
            baseDevice.setPayload(newReceivedPayload);
            std::string msg = baseDevice.getMsg(newReceivedPayload);
            int8_t uid = baseDevice.getUid(newReceivedPayload);
            if (valid)
            {
                setMostRecentMsg(msg, uid);
                jsonPayload = baseDevice.getJsonPayload();
                Serial.println("[Payload Received]");
                Serial.println(jsonPayload.c_str());
                beep3();
                clearLCD();
                displayText("From: " + String(uid), 0);
                scrollText(String(msg.c_str()), 1, 200);
                // baseDevice.printAckBucket();
            }
            // handleReceivedPayload(receivedPayload);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Check for new LoRa messages at a regular interval
    }
}

void _SerialListenTask(void *pvParameters)
{
    JsonDocument doc;

    while (1)
    {
        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n');
            DeserializationError error = deserializeJson(doc, input.c_str());

            Serial.print("Received from Base Station : ");
            Serial.println(input.c_str());

            if (!error)
            {
                int userId = doc["userId"] | -1;

                if (doc["data"].is<int>())
                {
                    Serial.println("PMSG");
                    int numberData = doc["data"];
                    baseDevice.createPmsg(numberData, userId, 0);
                    std::vector<int8_t> *payloadVector = new std::vector<int8_t>(baseDevice.getPayload());
                    std::string msg = baseDevice.getMsg(*payloadVector);
                    int8_t uid = baseDevice.getUid(*payloadVector);
                    xQueueSend(loraQueue, &payloadVector, portMAX_DELAY);
                    beep2();
                    clearLCD();
                    displayText("From: " + String(uid), 0);
                    scrollText(String(msg.c_str()), 1, 200);
                }
                else if (doc["data"].is<const char *>())
                {
                    Serial.println("CMSG");
                    std::string stringData = doc["data"].as<const char *>();
                    baseDevice.createCmsg(stringData, userId, 0);
                    std::vector<int8_t> *payloadVector = new std::vector<int8_t>(baseDevice.getPayload());
                    std::string msg = baseDevice.getMsg(*payloadVector);
                    int8_t uid = baseDevice.getUid(*payloadVector);
                    xQueueSend(loraQueue, &payloadVector, portMAX_DELAY);
                    beep2();
                    clearLCD();
                    displayText("From: " + String(uid), 0);
                    scrollText(String(msg.c_str()), 1, 700);
                }
                else
                {
                    Serial.println("Unknown data type!");
                }
            }
            else
            {
                Serial.println("JSON Parse Error!");
            }
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void _LoRaSendTask(void *pvParameters)
{
    for (;;)
    {
        std::vector<int8_t> *receivedVec;
        if (xQueueReceive(loraQueue, &receivedVec, portMAX_DELAY))
        {
            bool pass = baseDevice.setPayload(*receivedVec);
            printf("Payload set: %s\n", pass ? "[true]" : "[false]");
            baseDevice.loraSend();
            beep2();
            delete receivedVec; // free when done
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed
    }
}

void _LCDTask(void *pvParameters)
{
    // Combind Most Recent Messege and User ID
    for (;;)
    {

        Serial.println("[LCD Task Started]");

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void loop() {}

// {"userId": 77,"type": "pmsg","data": 12}
// {"userId": 77,"type": "cmsg","data": "Hi man rajuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu"}

// {"userId": 77,"type": "cmsg","data": "Hi man raju"}

void setMostRecentMsg(std::string msg, int8_t uid)
{
    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(100)))
    {
        mostRecentMsg = msg;
        mostRecentUid = uid;
        xSemaphoreGive(xSemaphore);
    }
}

void setupBuzzer()
{
    pinMode(BUZZER_PIN, OUTPUT);
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(BUZZER_PIN, PWM_CHANNEL);
    noTone();
}

void beep()
{
#ifdef ACTIVE_BUZZER
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
#else
    ledcWriteTone(PWM_CHANNEL, 1000);
    delay(100);
    noTone();
#endif
}

void beep2()
{
    beep();
    delay(100);
    beep();
}

void beep3()
{
#ifdef ACTIVE_BUZZER
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
#else
    ledcWriteTone(PWM_CHANNEL, 500);
    delay(500);
    noTone();
    delay(100);
    ledcWriteTone(PWM_CHANNEL, 500);
    delay(500);
    noTone();
#endif
}

void noTone()
{
    ledcWrite(PWM_CHANNEL, 0);
}

// Function to display a message on a specific row
void displayText(String message, int row)
{
    if (row >= 0 && row < LCD_ROWS)
    {
        lcd.setCursor(0, row); // Set cursor to start of specified row
        lcd.print(message);
        // Clear remaining characters in the row to avoid leftover text
        for (int i = message.length(); i < LCD_COLUMNS; i++)
        {
            lcd.print(" ");
        }
    }
}

// Function to clear the LCD
void clearLCD()
{
    lcd.clear();
}

// Function to display a scrolling message on a specific row
void scrollText(String message, int row, int delayTime)
{
    if (row >= 0 && row < LCD_ROWS)
    {
        // Pad message with spaces for smooth scrolling
        String paddedMessage = message + " ";
        for (int i = 0; i <= paddedMessage.length() - LCD_COLUMNS; i++)
        {
            lcd.setCursor(0, row);
            lcd.print(paddedMessage.substring(i, i + LCD_COLUMNS));
            vTaskDelay(pdMS_TO_TICKS(delayTime)); // Delay for scrolling effect
        }
    }
}

void setupLCD()
{
    // Initialize I2C with default pins (GPIO 21 SDA, GPIO 22 SCL)
    // For alternate pins (e.g., TTGO T-Call), use: Wire.begin(18, 19);
    Wire.begin();

    // Initialize LCD
    lcd.init();
    lcd.backlight(); // Turn on backlight
    lcd.clear();     // Clear the display
}