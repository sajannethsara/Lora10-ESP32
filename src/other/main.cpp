// #include "BLE.h"
// #include "Protocol.h"
//  #include <NimBLEDevice.h>

// MyBLEConnection ble;
// UserDevicePayload userDevice(108);

// void setup()
// {
//     Serial.begin(115200);
//     // Initialize BLE connection
//     ble.StartBLE();

//     // Set up other necessary components
// }

// void loop()
// {   
//     userDevice.setInboxNew("1:Hello Raju");
//     userDevice.setInboxNew("2:Hello Rajuuuu");
//     bool a = ble.ServeInboxChunks(2, 10, userDevice.getInbox());
//     userDevice.printInbox();
//     delay(2000); // Adjust delay as needed
// }

// /*
// Base Station code ek final karanna one.
//   -  msg resive karanna one.-app ekt json denna one. - adala ewa newata forword karann one.
//   -  msg send karanna one. - from app commanads walata.

// Intermediate Device code ek final karanna one.
//   - msg resive krala adala ewa forword karann one.
//   - predifined msg LCD eke pennann one.
//   - button task ekk hadann one.
//   - msg send wenna hadanna one.

// User Device code ek final karanna one.
//   - button task ek weda karanna one.
//   - OLED ek maru wenna one.
//   - msg resive krala inbox walata add karann one.
//   - adala msg forword karann one.
//   - gps yawanna one.
//   -  b

// */