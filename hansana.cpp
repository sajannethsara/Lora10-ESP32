#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <U8g2lib.h>
#define MODE_BTN   34
#define UP_BTN     25
#define DOWN_BTN   35
#define OK_BTN     32

#define H_FONT u8g2_font_ncenB14_tr
#define P_FONT u8g2_font_6x10_tr

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0 /* rotation /, /=SDA=/21, /=SCL=*/22);

SemaphoreHandle_t xSemaphore;

struct InMsg {
  String messageContent;
};
InMsg receivedMessages[10];  // up to 10 in inbox
const char* emergencyMessages[10] = {
  "I'm OK",
  "I need water",
  "I need food",
  "I need medical assistance",
  "I'm lost, send help",
  "I am injured",
  "There is a fire nearby",
  "I need shelter",
  "I am trapped, please rescue",
  "Send my location to the rescue team"
};
int messageCount = 0;      
int page = 0;              // Active screen index (0…4)
uint8_t states[5][2] = {0};         // [screen][0=selection][1=OK‑flag]

struct OutMsg {
  uint8_t  type;
  uint16_t transactionID;
  uint8_t  msgID;
};

void renderUI();
void renderWelcome();
void renderInbox();
void renderSend();
void renderCompass();
void renderBluetooth();

void h_mode();

void h_upWelcome();
void h_upInbox();
void h_upSend();
void h_upCompass();
void h_upBluetooth();

void h_downWelcome();
void h_downInbox();
void h_downSend();
void h_downCompass();
void h_downBluetooth();

void h_okWelcome();
void h_okInbox();
void h_okSend();
void h_okCompass();
void h_okBluetooth();

// ─── Dispatch‑table Setup ────────────────────────────────────
using Hfunc = void(*)();
static Hfunc modeH    = h_mode;
static Hfunc upH[5]   = { h_upWelcome, h_upInbox, h_upSend, h_upCompass, h_upBluetooth };
static Hfunc downH[5] = { h_downWelcome, h_downInbox, h_downSend, h_downCompass, h_downBluetooth };
static Hfunc okH[5]   = { h_okWelcome,  h_okInbox,  h_okSend,  h_okCompass,  h_okBluetooth };

// ─── ButtonTask (dispatch‑table method) ──────────────────────
void ButtonTask(void *pv) {
  const uint32_t debounce = 200;
  static uint32_t lastDebounce = 0;

  while (1) {
    uint32_t now = millis();

    // Debounce window
    if (now - lastDebounce > debounce) {
      // MODE → cycle screens
      if (digitalRead(MODE_BTN) == HIGH) {
        modeH();
        lastDebounce = now;
        vTaskDelay(50 / portTICK_PERIOD_MS);
        continue;
      }

      // UP → per‑screen UP handler
      if (digitalRead(UP_BTN) == HIGH) {
        upH[currentScreen]();
        lastDebounce = now;
        vTaskDelay(50 / portTICK_PERIOD_MS);
        continue;
      }

      // DOWN → per‑screen DOWN handler
      if (digitalRead(DOWN_BTN) == HIGH) {
        downH[currentScreen]();
        lastDebounce = now;
        vTaskDelay(50 / portTICK_PERIOD_MS);
        continue;
      }

      // OK → per‑screen OK handler
      if (digitalRead(OK_BTN) == HIGH) {
        okH[currentScreen]();
        lastDebounce = now;
        vTaskDelay(50 / portTICK_PERIOD_MS);
        continue;
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ─── Handler Definitions ─────────────────────────────────────
// MODE: advance screen index 0→4
void h_mode() {
  if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
    currentScreen = (currentScreen + 1) % 5;
    xSemaphoreGive(xSemaphore);
  }
}

// UP
void h_upWelcome()   { /* no action */ }
void h_upInbox()     {
  if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
    if (states[1][0] > 0){
        states[1][0]--;
    }
    xSemaphoreGive(xSemaphore);
  }
}
void h_upSend()      {
  if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
    if (states[2][0] < 9){
        states[2][0]++;  // 10 presets
    }
    xSemaphoreGive(xSemaphore);
  }
}
void h_upCompass()   { /* future breadcrumb‐index logic */ }
void h_upBluetooth() { /* no action */ }

// DOWN
void h_downWelcome()   { /* no action */ }
void h_downInbox()     {
  if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
    if (states[1][0] < messageCount - 1){
        states[1][0]++;
    }
    xSemaphoreGive(xSemaphore);
  }
}
void h_downSend()      {
  if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
    if (states[2][0] > 0){
        states[2][0]--;
    }
    xSemaphoreGive(xSemaphore);
  }
}
void h_downCompass()   { /* future breadcrumb‐index logic */ }
void h_downBluetooth() { /* no action */ }

// OK
void h_okWelcome()    { /* no action */ }
void h_okInbox()      {
  if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
    states[1][1] = 1;  // flag “open detail”
    xSemaphoreGive(xSemaphore);
  }
}
void h_okSend()       {
  if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
    states[2][1] = 1;  // flag “sending…”
    /*OutMsg msg = {
      PayloadBuilder::TYPE_PMSG,
      transmissionID++,
      states[2][0]
    };
    xQueueSend(loraQueue, &msg, portMAX_DELAY);*/
    xSemaphoreGive(xSemaphore);
  }
}
void h_okCompass()    { /* toggle reverse‑nav mode */ }
void h_okBluetooth()  { /* start BLE pairing/transmit */ }


void DisplayTask(void *pv) {
  while (1) {
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
      renderUI();
      xSemaphoreGive(xSemaphore);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void renderUI() {
  switch (currentScreen) {
    case 0: renderWelcome();   break;
    case 1: renderInbox();     break;
    case 2: renderSend();      break;
    case 3: renderCompass();   break;
    case 4: renderBluetooth(); break;
    default: renderWelcome();  break;
  }
}

// Screen 0: Welcome
void renderWelcome() {
  u8g2.clearBuffer();
  u8g2.setFont(H_FONT);
  u8g2.drawStr(0, 10, "WELCOME");
  u8g2.drawLine(0, 11, 127, 11);
}

// Screen 1: Inbox
void renderInbox() {
  u8g2.clearBuffer();
  u8g2.setFont(H_FONT);    
  u8g2.drawStr(0, 10, "INBOX");
  u8g2.drawLine(0, 11, 127, 11);

  u8g2.setFont(P_FONT);
  if (messageCount == 0) {
    u8g2.drawStr(0, 25, "No messages");
  } else {
    for (int i = 0; i < messageCount && i < 5; i++) {
      int y = 25 + i * 10;
      if (i == states[1][0]) {
        u8g2.drawBox(0, y - 8, 128, 10);
        u8g2.setDrawColor(0);
        u8g2.drawStr(2, y, receivedMessages[i].messageContent.c_str());
        u8g2.setDrawColor(1);
      } else {
        u8g2.drawStr(2, y, receivedMessages[i].messageContent.c_str());
      }
    }
    // Show detail‑flag if OK pressed
    if (states[1][1]) {
      u8g2.drawBox(0, 58, 128, 6);
      u8g2.setDrawColor(0);
      u8g2.drawStr(2, 63, "OK → Open Msg");
      u8g2.setDrawColor(1);
    }
  }
  u8g2.sendBuffer();
}

// Screen 2: Send
void renderSend() {
  u8g2.clearBuffer();
  u8g2.setFont(H_FONT);    
  u8g2.drawStr(0, 10, "SEND");
  u8g2.drawLine(0, 11, 127, 11);

  u8g2.setFont(P_FONT);
  u8g2.drawStr(0, 24, "Selected Msg:");
  const char* txt = emergencyMessages[ states[2][0] ];
  u8g2.drawStr(0, 36, txt);

  // “sending” indicator
  if (states[2][1]) {
    u8g2.drawStr(0, 50, "Sending...");
  }
  u8g2.sendBuffer();
}

// Screen 3: Compass
void renderCompass() {
  u8g2.clearBuffer();
  u8g2.setFont(H_FONT);    
  u8g2.drawStr(0, 10, "COMPASS");
  u8g2.drawLine(0, 11, 127, 11);
  u8g2.setFont(P_FONT);    
  u8g2.drawStr(0, 30, "Heading placeholder");
  u8g2.sendBuffer();
}

// Screen 4: Bluetooth
void renderBluetooth() {
  u8g2.clearBuffer();
  u8g2.setFont(H_FONT);    
  u8g2.drawStr(0, 10, "BLUETOOTH");
  u8g2.drawLine(0, 11, 127, 11);
  u8g2.setFont(P_FONT);    
  u8g2.drawStr(0, 30, "No BT operations");
  u8g2.sendBuffer();
}

void setup() {
  // 1) Serial for debug
  Serial.begin(115200);

  // 2) Initialize display
  u8g2.begin();

  // 3) Configure buttons as inputs with pull‑ups
  pinMode(MODE_BTN,   INPUT);
  pinMode(UP_BTN,     INPUT);
  pinMode(DOWN_BTN,   INPUT);
  pinMode(OK_BTN,     INPUT);

  // 4) Create the mutex for task synchronization
  xSemaphore = xSemaphoreCreateMutex();

  // 5) Spawn your two FreeRTOS tasks
  xTaskCreatePinnedToCore(
    ButtonTask,         // task function
    "ButtonTask",       // name
    2048,               // stack size (bytes)
    NULL,               // parameter
    tskIDLE_PRIORITY+1, // priority
    NULL,               // task handle
    1                   // run on core 1
  );
  xTaskCreatePinnedToCore(
    DisplayTask,
    "DisplayTask",
    4096,
    NULL,
    tskIDLE_PRIORITY+1,
    NULL,
    1
  );
}

void loop(){
}