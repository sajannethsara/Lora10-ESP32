// Final user device code:

#include <Arduino.h>
#include <string.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

#include "payload_builder.h"
#include <U8g2lib.h>

// #include <Arduino.h>
// #include <U8g2lib.h>
const std::string emergencyMessages[10] = {
    "I'm OK",
    "I need water",
    "I need food",
    "I need medical assistance",
    "I'm lost, send help",
    "I am injured",
    "There is a fire nearby",
    "I need shelter",
    "I am trapped, please rescue",
    "Send my location to the rescue team"};

const std::string baseMessages[10] = {
  "All clear",
  "Evacuate immediately",
  "Proceed to checkpoint",
  "Remain calm",
  "Await further instructions",
  "Medical team is en route",
  "Rescue team dispatched",
  "Help is arriving",
  "Situation under control",
  "Mission accomplished"
};


// Fonts
#define H_FONT u8g2_font_ncenB08_tr
#define P_FONT u8g2_font_6x10_tr

// Button Pins
#define OK_BTN 33
#define MODE_BTN 32
#define UP_BTN 25
#define DOWN_BTN 27

#define LORA_SS 5            // LoRa SPI CS pin
#define LORA_RST 14          // LoRa reset pin
#define LORA_DIO0 26         // LoRa IRQ pin
#define MAX_LORA_MESSAGES 30 // Tracks which row is active (0, 1, or 2)

#define R1 10000.0  // 10kΩ resistor
#define R2 10000.0  // 10kΩ resistor
#define REF_VOLTAGE 3.3  // ESP32 ADC reference voltage
#define ADC_PIN 34  // GPIO 34 for analog input (change if needed)

// Function to get battery voltage
float getBatteryVoltage() {
  int rawValue = analogRead(ADC_PIN);  // Read ADC value
  // if (rawValue == 0 || rawValue == 4095) {  // Check if ADC value is at extremes
  //   return -1.0;  // Indicate an error or disconnected state
  // }
  float measuredV = (rawValue * REF_VOLTAGE) / 4095.0;  // Convert to voltage
  float batteryV = measuredV * ((R1 + R2) / R2);  // Reverse voltage divider effect
  return batteryV;
}

// Function to calculate battery percentage
int getBatteryPercentage(float voltage) {
  // if (voltage < 0) {
  //   return -1;  // Error state
  // }
  float minVoltage = 3.2;  // Battery empty voltage
  float maxVoltage = 4.2;  // Fully charged voltage
  float percentage = ((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100.0;
  percentage = constrain(percentage, 0, 100);  // Limit between 0-100%
  return (int)percentage;
}

// U8G2 for I2C Display
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// 3x3 Array Initialization
uint8_t states[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
uint8_t selectedRow = 0;
uint16_t transmissionID = 56789;
int RSSI = -50;
int signalStrength = 5;
struct LoRaMessage
{
  int8_t senderID;
  int16_t transactionID;
  std::string messageContent;
};

LoRaMessage receivedMessages[MAX_LORA_MESSAGES];
int messageCount = 0; // To track the number of received messages
int battPercent =  0;
// Debounce Handling
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;

// FreeRTOS handles
SemaphoreHandle_t xSemaphore;
QueueHandle_t loraQueue; // Queue to handle LoRa message requests
PayloadBuilder builder;

// UI Render Functions
void renderUI();
void renderInbox();
void renderSend();
void renderWelcome();

void ButtonTask(void *pvParameters);
void DisplayTask(void *pvParameters);
void LoRaTask(void *pvParameters);
void LoRaReceiveTask(void *pvParameters);
void updateInbox(int8_t senderID, int16_t transactionID, std::string messageContent);
int getSignalStrength()
    {
      if (RSSI > -60)
        return 4;
      else if (RSSI > -80)
        return 3;
      else if (RSSI > -100)
        return 2;
      else if (RSSI > -120)
        return 1;
      else
        return 0;
    }
void setup()
{
  Serial.begin(9600);
  builder.configure_device(0x01, 0x02);
  u8g2.begin();
  u8g2.clearBuffer();
  // Initialize Buttons with Pull-down Resistors
  pinMode(MODE_BTN, INPUT_PULLDOWN);
  pinMode(UP_BTN, INPUT_PULLDOWN);
  pinMode(DOWN_BTN, INPUT_PULLDOWN);
  pinMode(OK_BTN, INPUT_PULLDOWN);

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6))
  { // Set frequency to 433 MHz
    Serial.println("LoRa init failed. Check connections.");
    while (1)
      ;
  }
  Serial.println("LoRa init succeeded.");

  // Create a semaphore for shared access between tasks
  xSemaphore = xSemaphoreCreateMutex();
  loraQueue = xQueueCreate(5, sizeof(int)); // Queue can hold 5 integers (message IDs)

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(ButtonTask, "Button Task", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(DisplayTask, "Display Task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(LoRaTask, "LoRaTask", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(LoRaReceiveTask, "LoRaReceiveTask", 2048, NULL, 2, NULL, 0); // Add this line to create the receive task on core 0
}

void loop()
{
  // Not needed as FreeRTOS manages tasks
}

void ButtonTask(void *pvParameters)
{
  while (1)
  {
    if (millis() - lastDebounceTime > debounceDelay)
    {
      if (digitalRead(MODE_BTN) == HIGH)
      {
        Serial.println("MODE");
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
        {
          selectedRow = (selectedRow + 1) % 3;
          for (uint8_t i = 0; i < 3; i++)
          {
            states[i][0] = (i == selectedRow) ? 1 : 0;
          }
          xSemaphoreGive(xSemaphore);
        }
        lastDebounceTime = millis();
      }

      if (digitalRead(UP_BTN) == HIGH)
      {
        Serial.println("UP");
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
        {
          if (selectedRow == 1)
          {
            Serial.print(messageCount);
            if (states[selectedRow][1] <= messageCount)
            {
              Serial.print(messageCount);
              states[selectedRow][1]++;
            }
          }
          else if (states[selectedRow][1] < 10)
          {
            states[selectedRow][1]++;
          }
          xSemaphoreGive(xSemaphore);
        }
        lastDebounceTime = millis();
      }

      if (digitalRead(DOWN_BTN) == HIGH)
      {
        Serial.println("DOWN");
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
        {
          if (states[selectedRow][1] > 0)
          {
            states[selectedRow][1]--;
            Serial.println(messageCount);
            Serial.println(states[selectedRow][1]);
          }
          xSemaphoreGive(xSemaphore);
        }
        lastDebounceTime = millis();
      }

      if (digitalRead(OK_BTN) == HIGH)
      {
        Serial.println("OK");
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
        {
          states[selectedRow][2] = !states[selectedRow][2];
          if (selectedRow == 2 && states[selectedRow][2] == 1)
          {
            int messageID = states[selectedRow][1];
            xQueueSend(loraQueue, &messageID, portMAX_DELAY);
            states[selectedRow][2] = 0; // Send message to LoRa task
          }
          xSemaphoreGive(xSemaphore);
        }
        lastDebounceTime = millis();
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void DisplayTask(void *pvParameters)
{
  while (1)
  {
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
    {
      renderUI();
      xSemaphoreGive(xSemaphore);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Refresh rate
  }
}

void renderUI()
{
  if (selectedRow == 0)
  {
    renderWelcome();
  }
  else if (selectedRow == 1)
  {
    renderInbox();
  }
  else if (selectedRow == 2)
  {
    renderSend();
  }
}
void renderInbox()
{
  u8g2.firstPage();
  do
  {
    if (states[1][2] == 1)
    {
      u8g2.setFont(H_FONT);
      u8g2.drawStr(0, 10, "Message:");
      if (messageCount > 0 && states[1][1] < messageCount)
      {
        String msg = String(receivedMessages[states[1][1]].messageContent.c_str());
        int lineHeight = 10;
        int maxCharsPerLine = 21;
        int y = 22;
        for (unsigned int i = 0; i < msg.length(); i += maxCharsPerLine)
        {
          String line = msg.substring(i, i + maxCharsPerLine);
          u8g2.drawStr(0, y, line.c_str());
          y += lineHeight;
        }
      }
    }
    else
    {
      u8g2.setFont(H_FONT);
      int w=u8g2.getStrWidth("INBOX");
      int x=(128-w)/2;
      u8g2.drawStr(x, 10, "INBOX");
      u8g2.drawLine(0, 11, 127, 11);
      if (messageCount == 0)
      {
        u8g2.setFont(P_FONT);
        u8g2.drawStr(0, 22, "No messages");
      }
      else
      {
        int total = messageCount;
        int visible = (total < 4) ? total : 4;
        int selected = states[1][1];
        int startIndex = selected - visible / 2;
        if (startIndex < 0)
          startIndex = 0;
        if (startIndex > total - visible)
          startIndex = total - visible;
        int startY = 22;
        int lineHeight = 12;
        for (int i = startIndex; i < startIndex + visible; i++)
        {
          int y = startY + (i - startIndex) * lineHeight;
          if (i == selected)
          {
            u8g2.drawFrame(0, y - 9, 128, 12);
          }
          String preview = String(receivedMessages[i].messageContent.c_str());
          if (preview.length() > 20)
          {
            preview = preview.substring(0, 20);
          }
          u8g2.setFont(P_FONT);
          u8g2.drawStr(0, y, preview.c_str());
        }
      }
    }
  } while (u8g2.nextPage());
}

/*void renderInbox() {
  u8g2.clearBuffer();
  u8g2.setFont(H_FONT);
  u8g2.drawStr(0, 10, "INBOX");
  u8g2.drawLine(0, 11, 127, 11);

  char buffer[10];
  itoa(states[selectedRow][1], buffer, 10);

  u8g2.setFont(P_FONT);
  // u8g2.drawStr(0, 21, "SELECTED MSG:");
  // u8g2.drawStr(0, 31, buffer);

  // if (states[selectedRow][2] == 1) {
  //   u8g2.drawStr(0, 41, "OK Pressed...");
  // }
  for (int i = 0; i < messageCount; i++) {
    // Only display up to 5 messages to fit on the screen
    if (i < 5) {
      u8g2.drawStr(0, 41 + (i * 10), receivedMessages[i].messageContent.c_str());
    }
  }

  // If no messages are received
  if (messageCount == 0) {
    u8g2.drawStr(0, 41, "No messages");
  }

  u8g2.sendBuffer();
}*/

void renderSend()
{
  u8g2.firstPage();
  do
  {
    u8g2.setFont(H_FONT);
    int w=u8g2.getStrWidth("SEND");
    int x=(128-w)/2;
    u8g2.drawStr(x, 10, "SEND");
    u8g2.drawLine(0, 11, 127, 11);
    if (states[2][2] == 1)
    {
      u8g2.setFont(P_FONT);
      u8g2.drawStr(0, 32, "Sending...");
    }
    else
    {
      int startY = 22;
      int lineHeight = 12;
      int total = 10;
      int visible = 4;
      int selected = states[2][1];
      int startIndex = selected - visible / 2;
      if (startIndex < 0)
        startIndex = 0;
      if (startIndex > total - visible)
        startIndex = total - visible;
      for (int i = startIndex; i < startIndex + visible; i++)
      {
        int y = startY + (i - startIndex) * lineHeight;
        if (i == selected)
        {
          u8g2.drawFrame(0, y - 9, 128, 12);
        }
        String preview = String(emergencyMessages[i].c_str());
        if (preview.length() > 20)
        {
          preview = preview.substring(0, 20);
        }
        u8g2.setFont(P_FONT);
        u8g2.drawStr(0, y, preview.c_str());
      }
    }
  } while (u8g2.nextPage());
}

/*void renderSend() {
  u8g2.clearBuffer();
  u8g2.setFont(H_FONT);
  u8g2.drawStr(0, 10, "SEND");
  u8g2.drawLine(0, 11, 127, 11);

  char buffer[10];
  itoa(states[selectedRow][1], buffer, 10);
  u8g2.setFont(P_FONT);
  u8g2.drawStr(0, 21, "SELECTED P_MSG:");
  u8g2.drawStr(0, 31, buffer);

  if (states[selectedRow][2] == 1) {
    u8g2.drawStr(0, 41, "OK Pressed...");
  }

  u8g2.sendBuffer();
}*/

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void renderWelcome()
{
  u8g2.firstPage();
  do
  {
    // Draw "WELCOME" centered at the top
    u8g2.setFont(H_FONT);
    int w = u8g2.getStrWidth("WELCOME");
    int x = (128 - w) / 2;
    u8g2.drawStr(x, 10, "WELCOME");
    u8g2.drawLine(0, 11, 127, 11);

    // Function to convert RSSI to signal strength bars
    // Function to convert RSSI to signal strength bars
    // int getSignalStrength()
    // {
    //   if (RSSI > -60)
    //     return 4;
    //   else if (RSSI > -80)
    //     return 3;
    //   else if (RSSI > -100)
    //     return 2;
    //   else if (RSSI > -120)
    //     return 1;
    //   else
    //     return 0;
    // }

    // Draw signal strength bars on the left side

    int signalStrength = getSignalStrength(); // Uses global RSSI

    int sigX = 10, sigY = 55, barW = 5, spacing = 3;
    for (int i = 0; i < 5; i++)
    {
      int barH = (i + 1) * 4;
      if (i < signalStrength)
        u8g2.drawBox(sigX + i * (barW + spacing), sigY - barH, barW, barH);
      else
        u8g2.drawFrame(sigX + i * (barW + spacing), sigY - barH, barW, barH);
    }

    // drawSignalStrength();
    float batteryVoltage = getBatteryVoltage();
    int batteryPercentage = getBatteryPercentage(batteryVoltage);
    if(batteryPercentage != -1){
      battPercent = batteryPercentage;
    }
  
    /*// Draw signal strength bars on the left side
    int sigX = 10, sigY = 55, barW = 5, spacing = 3;
    for (int i = 0; i < 5; i++) {
      int barH = (i + 1) * 4;
      if (i < signalStrength)
        u8g2.drawBox(sigX + i * (barW + spacing), sigY - barH, barW, barH);
      else
        u8g2.drawFrame(sigX + i * (barW + spacing), sigY - barH, barW, barH);
    }*/

    // Draw battery icon on the right side
    // int battPercent = batteryPercentage; // battPercent is the variable for Dinira. If it is 50, percentage is 50.
    int fillWidth = (battPercent / 100.0) * 20;
    int batX = 90, batY = 42;
    u8g2.drawFrame(batX, batY, 24, 12);
    u8g2.drawBox(batX + 24, batY + 3, 3, 6);
    u8g2.drawBox(batX + 2, batY + 2, fillWidth, 8);
    char battStr[5];
    snprintf(battStr, sizeof(battStr), "%d%%", (int)battPercent);
    u8g2.setFont(u8g2_font_6x10_tf);
    int textWidth = u8g2.getStrWidth(battStr);
    u8g2.drawStr(batX + (24 - textWidth) / 2, batY + 18, battStr);

  } while (u8g2.nextPage());
}

/*void renderWelcome() {
  u8g2.clearBuffer();
  u8g2.setFont(H_FONT);
  u8g2.drawStr(0, 10, "WELCOME");
  u8g2.drawLine(0, 11, 127, 11);
  u8g2.setFont(H_FONT);
  u8g2.drawStr(0, 21, "RSSI:");
  char rssiBuffer[10];
  itoa(RSSI, rssiBuffer, 10);
  u8g2.drawStr(50, 21, rssiBuffer);
  u8g2.sendBuffer();
}*/

void LoRaTask(void *pvParameters)
{
  int receivedMessageID;

  for (;;)
  {
    if (xQueueReceive(loraQueue, &receivedMessageID, portMAX_DELAY))
    {
      // Send the message via LoRa
      Serial.print("Sending LoRa Message with ID: ");
      Serial.println(receivedMessageID); // 0 - 9
      std::vector<uint8_t> pMsgPayload = builder.create_p_msg_payload(transmissionID, receivedMessageID + 1);
      transmissionID++;
      LoRa.beginPacket();
      // LoRa.write(pMsgPayload., pMsgPayload.size());
      for (size_t i = 0; i < pMsgPayload.size(); i++)
      {
        LoRa.write(pMsgPayload[i]);
      }
      LoRa.endPacket();

      Serial.println("Message sent!");
    }
  }
}
// now I need to add another task for listning for lora signals and when resived I need to process and store data in a new struct array

void LoRaReceiveTask(void *pvParameters)
{
  while (1)
  {
    int packetSize = LoRa.parsePacket();
    if (packetSize)
    {
      std::vector<uint8_t> receivedPayload;
      RSSI = LoRa.packetRssi();
      Serial.print("Received packet: ");
      while (LoRa.available())
      {
        receivedPayload.push_back(LoRa.read());
      }
      // handleReceivedPayload(receivedPayload);
      uint8_t payloadType = builder.identify_type_and_check_checksum(receivedPayload);
      if (payloadType != 101)
      {
        if (payloadType == 2)
        {
          PayloadBuilder::PMsgData pMsgData = builder.decode_p_msg_payload(receivedPayload);
          PayloadBuilder::PayloadDetails payloadDetails = builder.get_payload_details(receivedPayload);
          std::string pMessage = baseMessages[pMsgData.msgID];
          if(payloadDetails.destinationID == 0x01){
            updateInbox(payloadDetails.sourceID, payloadDetails.transmissionID, pMessage);
          }
          // updateInbox(payloadDetails.sourceID, payloadDetails.transmissionID, pMessage);
        }
        else if (payloadType == 3)
        {
          PayloadBuilder::CMsgData cMsgData = builder.decode_c_msg_payload(receivedPayload);
          PayloadBuilder::PayloadDetails payloadDetails = builder.get_payload_details(receivedPayload);
          updateInbox(payloadDetails.sourceID, payloadDetails.transmissionID, cMsgData.message);
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Check for new LoRa messages at a regular interval
  }
}

void updateInbox(int8_t senderID, int16_t transactionID, std::string messageContent)
{
  if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
  {
    if (messageCount < MAX_LORA_MESSAGES)
    {
      // Store the message data in the array
      receivedMessages[messageCount].senderID = senderID;
      receivedMessages[messageCount].transactionID = transactionID;
      receivedMessages[messageCount].messageContent = messageContent;

      // Increment the message count
      messageCount++;
    }
    else
    {
      // Shift all messages up to make room for the new message
      for (int i = 1; i < MAX_LORA_MESSAGES; i++)
      {
        receivedMessages[i - 1] = receivedMessages[i];
      }
      // Store the new message at the end
      receivedMessages[MAX_LORA_MESSAGES - 1].senderID = senderID;
      receivedMessages[MAX_LORA_MESSAGES - 1].transactionID = transactionID;
      receivedMessages[MAX_LORA_MESSAGES - 1].messageContent = messageContent;
    }
    for (int i = 0; i < messageCount; i++)
    {
      Serial.print("Message from: ");
      Serial.print(receivedMessages[i].senderID);
      Serial.print(" with transaction ID: ");
      Serial.print(receivedMessages[i].transactionID);
      Serial.print(" and message: ");
      Serial.println(receivedMessages[i].messageContent.c_str());
    }
    xSemaphoreGive(xSemaphore);
  } }


  void _CompassTask(void *pvParameters)
{
    for (;;)
    {
        // Run only when Compass mode is active and compass page is visible
        if (states[3][1] == 2 && page == 3)
        {
            // read sensors (use the same sensor-read helpers you already have)
            int16_t magX, magY, magZ;
            int16_t accX, accY, accZ;
            readMagnetometer(magX, magY, magZ);
            readAccelerometer(accX, accY, accZ);

            // ---- tilt compensation ----
            float axn = (float)accX;
            float ayn = (float)accY;
            float azn = (float)accZ;

            float norm = sqrt(axn * axn + ayn * ayn + azn * azn);
            if (norm == 0.0f) norm = 1.0f;
            axn /= norm;
            ayn /= norm;
            azn /= norm;

            // compute horizontal magnetometer components (tilt-compensated)
            float xh = (float)magX * azn - (float)magZ * axn;
            float yh = (float)magY * azn - (float)magZ * ayn;

            // heading in degrees (0..360, 0 = North)
            float newHeading = atan2(yh, xh) * 180.0f / PI;
            if (newHeading < 0.0f) newHeading += 360.0f;

            // ---- update shared global under mutex to avoid races with render/other tasks ----
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                heading = newHeading;       // keep existing global consistent
                compassHeading = newHeading; // this is what renderCompass() should read
                xSemaphoreGive(i2cMutex);
            }
            else
            {
                // If we can't grab the mutex, still update the volatile so UI gets approximate value
                heading = newHeading;
                compassHeading = newHeading;
            }
        }

        // match update rate of reverse nav (about 3 Hz)
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}