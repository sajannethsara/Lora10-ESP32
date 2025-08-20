/*
I need to make flutter app for my iOT project,

The ESP32 BLE Vector Sync interface enables a Flutter app to synchronize three std::vector<std::string> data structures (inbox, sendbox, gps) with an ESP32 server over Bluetooth Low Energy. The service (12345678-1234-5678-1234-56789abcdef0) exposes characteristics for each vector to read size (uint32_t), write index (uint32_t), read data (string), and receive notifications (new index) with an MTU of 120 bytes. The Flutter app connects to ESP32_Sync_Device, reads vector sizes, selects indices to retrieve data, subscribes to notifications for updates, and maintains local arrays to display real-time data in the UI
 */
