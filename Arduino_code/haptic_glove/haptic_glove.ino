/*********************************************************************
 
 SensoPatch Glove (https://github.com/angyanisa/SensoPatch)
 
 This sketch uses Adafruit Bluefruit nRF52 library: https://goo.gl/LdEx62
 for wireless communication between the central (glove) and multiple
 peripherals (vibration motors) of the SensoPatch haptic feedback system.

 This sketch will: 
   - Read analog values from 25 sensors on the glove via TDMA approach
   - Average the pressure readings and map sensors to vibration motors
     according to customizable compression modes
   - Check if the averaged pressure mapped to each vibration motor
     exceeds the threshold, send vibration intensity to peripherals accordingly
 
*********************************************************************/

#include <bluefruit.h>

/* Customizable number of vibration motors and compression modes.
 * This sketch provides examples for finger-focused and palm-focused
 * compression modes with 1 or 3 or 6 vibration motors.
 * Refer to the paper for more details.
 */
#define NUM_PERIPHERALS 6             // SELECT: 1 or 3 or 6
enum cpMode {FINGER = 0, PALM = 1};       
cpMode compressionMode = FINGER;      // SELECT: FINGER or PALM

typedef struct
{
  char name[16+1];
  uint16_t conn_handle;
  BLEClientUart bleuart;    // BLEUART client service for pressure readings
  BLEClientBas clientBas;   // Battery client service
} prph_info_t;              // Struct containing each peripheral info

prph_info_t prphs[BLE_MAX_CONNECTION];      // max number of connections is 20
uint8_t connection_num = 0;
const int numSensors = 25;
int pressureVals[numSensors] = {0};
int compressedVals[NUM_PERIPHERALS] = {0};
int outputVals[NUM_PERIPHERALS] = {0};

/* Thresholds for binary feedback of each vibration motor should be adjusted
 * depending on the number of vibration motors, compression modes,
 * and the fit of the glove. Experimental values need to be determined to avoid false feedback.
 * This example shows pressure reading thresholds for 1,3,6 vibration motors.
 * The first subarray is for finger-focused mode, the second subarray is for palm-focused mode.
 */
int buzz1Thresh[2][1] = { 150, 150 };
int buzz3Thresh[3][3] = { {300, 200, 450}, {150, 200, 200} };
int buzz6Thresh[2][6] = { {300, 150, 150, 150, 450, 200}, {300, 100, 350, 150, 200, 200} };

void setup() {
  Serial.begin(9600);
  
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);

  Serial.println("SensoPatch System Started");
  Serial.println("-----------------------------------------\n");

  // Initialize Bluefruit with max concurrent connections as Peripheral = 0, Central = NUM_PERIPHERALS
  Bluefruit.begin(0, NUM_PERIPHERALS);
  Bluefruit.setName("SensoPatch Glove");
  
  // Initilize peripheral pool
  for (uint8_t idx=0; idx<BLE_MAX_CONNECTION; idx++) {
    // Invalid all connection handle
    prphs[idx].conn_handle = BLE_CONN_HANDLE_INVALID;
    prphs[idx].clientBas.begin();
    prphs[idx].bleuart.begin();
  }

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Filter only accept bleuart service in advertising
   * - Don't use active scan (used to retrieve the optional scan response adv packet)
   * - Start(0) = will scan forever since no timeout is given
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80);       // in units of 0.625 ms
  Bluefruit.Scanner.filterUuid(BLEUART_UUID_SERVICE);
  Bluefruit.Scanner.useActiveScan(false);       // Don't request scan response data
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds
}

void loop() {
  /* The glove doesn't start reading pressure until all peripherals are connected.
   * Built-in LED turns from red to green once all peripherals successfully connected.
   */
  while (connection_num < NUM_PERIPHERALS) {
    if (digitalRead(LED_RED) || !digitalRead(LED_GREEN)) {
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GREEN, HIGH);
    }
  }
  while (connection_num == NUM_PERIPHERALS) {
    if (Bluefruit.Scanner.isRunning()) {
      Bluefruit.Scanner.stop();
    } 
    if (!digitalRead(LED_RED) || digitalRead(LED_GREEN)) {
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN, LOW);
    }
    read_sensor();
    send_buzzer();
  }
}

void read_sensor() {
  /* The indices of sensor array are provided in diagram on github */
  Serial.println("Reading Sensors . . . . .");
  
  /*-----------ROW 1-----------*/
  for (int i = 7; i < 11; i++) {
    pinMode(i, INPUT);
  }
  // Write 5V to R1
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);
  // Read the value from each column
  pressureVals[1] = analogRead(A1); // top index
  pressureVals[2] = analogRead(A2); // top mid
  pressureVals[3] = analogRead(A3);  // top ring
  pressureVals[4] = analogRead(A4);  // top pinky
  pressureVals[16] = analogRead(A0); // palm
  digitalWrite(6, LOW);

  /*-----------ROW 2-----------*/
  pinMode(6, INPUT);
  for (int i = 8; i < 11; i++) {
    pinMode(i, INPUT);
  }
  // Write 5V to R2
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  // Read the value from each column
  pressureVals[6] = analogRead(A1); // bot index
  pressureVals[7] = analogRead(A2); // bot mid
  pressureVals[8] = analogRead(A3);  // bot ring
  pressureVals[9] = analogRead(A4);  // bot pinky
  pressureVals[10] = analogRead(A0); // palm
  digitalWrite(7, LOW);

  /*-----------ROW 3-----------*/
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  // Write 5V to R3
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
  // Read the value from each column
  pressureVals[11] = analogRead(A1);  // palm
  pressureVals[12] = analogRead(A0);
  pressureVals[13] = analogRead(A2); 
  pressureVals[14] = analogRead(A3);
  pressureVals[15] = analogRead(A4);
  digitalWrite(8, LOW);

  /*-----------ROW 4-----------*/
  for (int i = 6; i < 9; i++) {
    pinMode(i, INPUT);
  }
  pinMode(10, INPUT);
  // Write 5V to R4
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);
  // Read the value from each column
  pressureVals[17] = analogRead(A1);  // palm
  pressureVals[18] = analogRead(A0);
  pressureVals[19] = analogRead(A2); 
  pressureVals[20] = analogRead(A3);
  pressureVals[21] = analogRead(A4);
  digitalWrite(9, LOW);
  
  /*-----------ROW 5-----------*/
  for (int i = 6; i < 10; i++) {
    pinMode(i, INPUT);
  }
  // Write 5V to R5
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  // Read the value from each column
  pressureVals[0] = analogRead(A0);  // top thumb
  pressureVals[5] = analogRead(A1);  // bot thumb
  pressureVals[22] = analogRead(A2);  // palm
  pressureVals[23] = analogRead(A3);
  pressureVals[24] = analogRead(A4);
  digitalWrite(10, LOW);


  /*-----------Compression Mode 1: finger-focused-----------
   *-----------Compression Mode 2: palm-focused-----------
   * Refer to the paper and the diagram on github for more information on
   * how pressure readings from 25 sensors averaged and mapped to 1,3,6 vibration motors.
   */
  switch (compressionMode) {
    case FINGER:
    {
      switch (NUM_PERIPHERALS) {
        case 1:
        {
          /* avg fingers */
          int total_fingers = 0;
          for (int i = 0; i < 5; i++) {
            total_fingers += (pressureVals[i] + pressureVals[i+5]);
          }
          compressedVals[0] = total_fingers / 10;
          break;
        }
        case 3:
        {
          /* avg thumb, middle, pinky */
          for (int i = 0; i < 3; i++) {
            compressedVals[i] = (pressureVals[2*i] + pressureVals[2*i+5]) / 2;
          }
          break;
        }
        case 6:
        {
          /* avg fingers */
          for (int i = 0; i < NUM_PERIPHERALS; i++) {
            compressedVals[i] = (pressureVals[i] + pressureVals[i+5]) / 2;
          }
          /* avg palm */
          int total_palm = 0;
          for (int i = 10; i < 25; i++) {
            total_palm += pressureVals[i];
          }
          compressedVals[5] = total_palm / 15;
          break;
        }
      }
      break;
    }
    case PALM:
    {
      switch (NUM_PERIPHERALS) {
        case 1:
        {
          /* avg palm */
          int total_palm = 0;
          for (int i = 10; i < 25; i++) {
            total_palm += pressureVals[i];
          }
          compressedVals[0] = total_palm / 15;
          break;
        }
        case 3:
        {
          /* left palm */
          compressedVals[0] = (pressureVals[10] + pressureVals[11] + pressureVals[16] + pressureVals[17]) / 4;
          /* middle palm */
          compressedVals[1] = (pressureVals[12] + pressureVals[13] + pressureVals[18] + pressureVals[19] + pressureVals[22]) / 5;
          /* right palm */
          compressedVals[2] = (pressureVals[14] + pressureVals[15] + pressureVals[20] + pressureVals[21] + pressureVals[23] + pressureVals[24]) / 6;
          break;
        }
        case 6:
        {
          /* thumb */
          compressedVals[0] = (pressureVals[0] + pressureVals[5]) / 2;
          /* upper finger */
          compressedVals[1] = (pressureVals[1] + pressureVals[2] + pressureVals[3] + pressureVals[4]) / 4;
          /* lower finger */
          compressedVals[2] = (pressureVals[6] + pressureVals[7] + pressureVals[8] + pressureVals[9]) / 4;
          /* left palm */
          compressedVals[3] = (pressureVals[10] + pressureVals[11] + pressureVals[16] + pressureVals[17]) / 4;
          /* middle palm */
          compressedVals[4] = (pressureVals[12] + pressureVals[13] + pressureVals[18] + pressureVals[19] + pressureVals[22]) / 5;
          /* right palm */
          compressedVals[5] = (pressureVals[14] + pressureVals[15] + pressureVals[20] + pressureVals[21] + pressureVals[23] + pressureVals[24]) / 6;
          break;
        }
      }
      break;
    }
  }

    
  /*-----------Binary Feedback-----------
   * Vibration intensity (range: 0-255) of the motors whose mapped averaged pressure
   * exceeds the threshold is set to 180. Other motors are set to 0 (no vibration). 
   */
  switch (NUM_PERIPHERALS) {
    case 1:
    {
      for (int i = 0; i < NUM_PERIPHERALS; i++) {
        outputVals[i] = (compressedVals[i] > buzz1Thresh[(int)compressionMode][i]) ? 180 : 0;
      }
      break;
    }
    case 3:
    {
      for (int i = 0; i < NUM_PERIPHERALS; i++) {
        outputVals[i] = (compressedVals[i] > buzz3Thresh[(int)compressionMode][i]) ? 180 : 0;
      }
      break;
    }
    case 6:
    {
      for (int i = 0; i < NUM_PERIPHERALS; i++) {
        outputVals[i] = (compressedVals[i] > buzz6Thresh[(int)compressionMode][i]) ? 180 : 0;
      }
      break;
    }
  }
}

void send_buzzer() {
  // iterate through every peripheral to send vibration feedback
  for(uint8_t id=0; id < NUM_PERIPHERALS; id++) {
    prph_info_t* peer = &prphs[id];
    if (peer->bleuart.discovered()) {
      int rawVal = compressedVals[(peer->name)[7] - '0'];
      Serial.print("Pressure: ")
      Serial.print(rawVal);
      Serial.print("\t");
      
      uint8_t vib = static_cast<uint8_t>(outputVals[(peer->name)[7] - '0']);
      peer->bleuart.write(&vib, sizeof(vib));
      Serial.print("Sent  ");
      Serial.print(vib);
      Serial.print("   to   ");
      Serial.println(peer->name);
    }
  }
  Serial.println();
}

/* Callback invoked when scanner picks up an advertising packet
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report) {
  Serial.println("************ Scan Callback ************");
  Bluefruit.Central.connect(report);
}

/* Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle) {
  Serial.println("************ Connect Callback ************");
  int id  = findConnHandle(BLE_CONN_HANDLE_INVALID);   // Find an available ID to use
  if ( id < 0 ) {
    Serial.println("Exceeded the number of connections !!!");
    return;
  }
  
  prph_info_t* peer = &prphs[id];
  peer->conn_handle = conn_handle;
  Bluefruit.Connection(conn_handle)->getPeerName(peer->name, sizeof(peer->name)-1);

  Serial.print("Connected to ");
  Serial.println(peer->name);

  connection_num++;
  Serial.print("Connection Num: ");
  Serial.println(connection_num);

  // check if this peripheral is our buzzer
  if (peer->name[0] != 'B') {
    Serial.println("NOT A BUZZER !");
    Bluefruit.disconnect(conn_handle);
    return;
  } else {
    // Discovering Battery Service
    if (peer->clientBas.discover(conn_handle)) {
      Serial.print("Battery Voltage: ");
      Serial.print(peer->clientBas.read() / 51.0);
      Serial.println(" V");
    } else {
      Serial.println("Battery NOT FOUND!");
      Bluefruit.disconnect(conn_handle);
      return;
    }
  
    // Discovering BLE UART Service
    if ( peer->bleuart.discover(conn_handle) ) {
      Serial.println("Found BLEUART service");
    } else {
      Serial.println("BLEUART Service NOT FOUND!");
      Bluefruit.disconnect(conn_handle);
      return;
    }
  }
  Bluefruit.Scanner.start(0);
}

/* Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle;
  (void) reason;

  Serial.println("************ Disconnect Callback ************");
  connection_num--;

  // Mark the ID as invalid
  int id  = findConnHandle(conn_handle);

  // Non-existant connection, something went wrong!
  if ( id < 0 ) {
    Serial.println("trying to disconnect non-existing connection !!!");
    return;
  }

  // Mark conn handle as invalid
  prphs[id].conn_handle = BLE_CONN_HANDLE_INVALID;

  Serial.print(prphs[id].name);
  Serial.println(" disconnected!");
  Serial.print("Connection Num: ");
  Serial.println(connection_num);

  delay(50);
  Bluefruit.Scanner.start(0);
}

/* Find the connection handle in the peripheral array
 * @param conn_handle Connection handle
 * @return array index if found, otherwise -1
 */
int findConnHandle(uint16_t conn_handle) {
  for(int id=0; id<BLE_MAX_CONNECTION; id++) {
    if (conn_handle == prphs[id].conn_handle) {
      return id;
    }
  }
  return -1;  
}
