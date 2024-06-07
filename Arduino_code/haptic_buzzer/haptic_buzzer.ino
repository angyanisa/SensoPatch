/*********************************************************************
 
 SensoPatch Vibration Patch (https://github.com/angyanisa/SensoPatch)
 
 This sketch uses Adafruit Bluefruit nRF52 library: https://goo.gl/LdEx62
 for wireless communication between the central (glove) and multiple
 peripherals (vibration motors buzzers) of the SensoPatch haptic feedback system.

 This sketch will read analog value of vibration intensity sent from the glove
 and output the vibration accordingly.

 A system with multiple vibration motors can use this sketch for
 every motor, with a change to the ID number in the peripheral name.
 
*********************************************************************/

#include <bluefruit.h>
#define MAX_PRPH_CONNECTION 1    // only connects to central (glove)
uint8_t connection_count = 0;

// BLE Service
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas blebas;   // battery level

void setup() {
  Serial.begin(9600);
  
  pinMode(5, OUTPUT);             // vibration motor
  digitalWrite(5, LOW);
  pinMode(VBAT_ENABLE, OUTPUT);   // to read LIPO battery level
  digitalWrite(VBAT_ENABLE, LOW);
  
  Serial.println("SensoPatch System Started");
  Serial.println("---------------------------\n");

  /* Setup the BLE LED to be enabled on CONNECT, otherwise red
   * Note: This is the default behaviour, but provided
   * here in case you want to control this LED manually via PIN 19
   */
  Bluefruit.autoConnLed(true);

  // Initialize Bluefruit with max concurrent connections as Peripheral = 1, Central = 0
  Bluefruit.begin(MAX_PRPH_CONNECTION, 0);
  Bluefruit.setTxPower(8);    // Check bluefruit.h for supported values

  /* TO DO: change the ID number for different vibration motor.
   * The ID has to be consecutive and start from 0.
   */
  Bluefruit.setName("BUZZER_0");    

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  
  bledfu.begin();
  bleuart.begin();

  // Read battery voltage
  unsigned int adcReading = analogRead(PIN_VBAT);
  double adcVoltage = (adcReading * 3.3) / 1024;
  double vBat = adcVoltage * 1510.0 / 510.0 * 1.12; // Voltage divider from Vbat to ADC and account for error.
  Serial.print("Battery Voltage = ");
  Serial.println(vBat);

  // Convert for transmission
  int batByte = vBat * 51;
  Serial.print("Battery Voltage Sent = ");
  Serial.println(batByte);

  // Send battery voltage
  blebas.begin();
  blebas.write(batByte);

  // Set up and start advertising
  startAdv();
}

void startAdv(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected) 
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void loop() {
  // output the vibration intensity sent from the glove
  if ( bleuart.available() ) {
    uint8_t vib;
    vib = (uint8_t) bleuart.read();
    Serial.print("vibration intensity = ");
    Serial.println(vib);
    analogWrite(5,(int) vib);
  }
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle) {
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);

  connection_count++;
  Serial.print("Connection count: ");
  Serial.println(connection_count);
  
  // Keep advertising if not reaching max
  if (connection_count < MAX_PRPH_CONNECTION) {
    Serial.println("Keep advertising");
    Bluefruit.Advertising.start(0);
  }
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle;
  (void) reason;

  analogWrite(5, 0);    // make sure the vibration is off once disconnected

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);

  connection_count--;
}
