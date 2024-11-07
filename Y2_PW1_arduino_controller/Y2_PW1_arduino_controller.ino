#include <ArduinoBLE.h>

#define pin_x_analogue A1
#define pin_y_analogue A0
#define pin_d_digital 2

int x = 0;
int y = 1;
int d = 0;
int debug = 3;
int32_t drive_power = 0;
int32_t turn_power = 0;
int32_t motor_power = 0;
int32_t prev_motor_power = 0;
char drive_direction = '0';
char turn_direction = '0';

unsigned long heartbeat_interval_ms = 750;
unsigned long heartbeat_tracker = 0;
unsigned long press_delay = 0;

int8_t command = 0;
int8_t prev_command = 0;
int32_t send = 0;
int32_t command_counter = 0;

int analog_centre = 2048;
int analog_limit = 4096;
int inner_deadzone = 200;
int outer_deadzone = 0;


void setup() {
  pinMode(pin_x_analogue, INPUT);
  pinMode(pin_y_analogue, INPUT);
  pinMode(pin_d_digital, INPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  Serial.begin(9600);
  delay(1000);
  while (!Serial);
  // initialize the Bluetooth® Low Energy hardware
  BLE.begin();

  Serial.println("Bluetooth® Low Energy Central");

  // start scanning for peripherals
  BLE.scanForUuid("6099");
}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    if (peripheral.localName() != "Grp") { // change to the correct device name
      return;
    }

    // stop scanning
    BLE.stopScan();

    controlCar(peripheral);

    // peripheral disconnected, start scanning again
    BLE.scanForUuid("6099");
  }
}

void controlCar(BLEDevice peripheral) {
  // connect to the peripheral
  Serial.println("Connecting ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
    int characteristicCount = peripheral.characteristicCount();
    Serial.print(characteristicCount);
    Serial.println(" characteristics discovered");

    for (int i = 0; i < characteristicCount; i++) {
      BLECharacteristic characteristic = peripheral.characteristic(i);
      Serial.print("Characteristic UUID: ");
      Serial.println(characteristic.uuid());
    }
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }


// retrieve the Car characteristic
  BLECharacteristic carCharacteristic = peripheral.characteristic(0);

  if (!carCharacteristic) {
    Serial.println("Peripheral does not have car characteristic!");
    delay(2000);
    peripheral.disconnect();
    return;
  } else if (!carCharacteristic.canWrite()) {
    Serial.println("Peripheral does not have a writable car characteristic!");
    delay(2000);
    peripheral.disconnect();
    return;
  }

  digitalWrite(LED_GREEN, HIGH);
  delay(15);
  digitalWrite(LED_GREEN, LOW);

  while (peripheral.connected()) {
    // while the peripheral is connected
      x = analogRead(pin_x_analogue);
      y = analogRead(pin_y_analogue);
      d = !digitalRead(pin_d_digital);
      if (debug == 1){
        Serial.print("X: ");
        Serial.print(x);
        Serial.print(", Y: ");
        Serial.print(y);
        Serial.print(", D: ");
        Serial.println(d);
        delay(10);
      }

      drive_power = (abs(y - 2048)) / 8 - 1;
      turn_power = (abs(x - 2048)) / 8 - 1;


      command = 0;
      send = 0;

      if (d && (millis() - press_delay > 1000)){
        command = 10;                                               // button press
        press_delay = 1000;
      } else {
        if (y > analog_centre + inner_deadzone){
          if (abs(x - analog_centre) < inner_deadzone) command = 1; // up
          else if (x > analog_centre + inner_deadzone) command = 2; // up right
          else if (x < analog_centre - inner_deadzone) command = 3; // up left
        } else if (y < analog_centre - inner_deadzone){
          if (abs(x - analog_centre) < inner_deadzone) command = 4; // down
          else if (x > analog_centre + inner_deadzone) command = 5; // down right
          else if (x < analog_centre - inner_deadzone) command = 6; // down left
        } else {
          if (x > analog_centre + inner_deadzone) command = 7;      // right
          else if (x < analog_centre - inner_deadzone) command = 8; // left
          else command = 9;                                         // stop
        }
      }

      send = command + (drive_power << 8) + (turn_power << 16) + (command_counter << 24);
      //printBinary(send);

      if ((command != prev_command) || (drive_power > 25) || (turn_power > 25)){
        
        Serial.print("Drive: ");
        Serial.print(drive_power);
        Serial.print(", Turn: ");
        Serial.print(turn_power);
        Serial.print(", command: ");
        Serial.println(command);

        send_command(carCharacteristic, send);
        command_counter++;

        Serial.print("0: ");
        Serial.print(send & 0xFF);
        Serial.print(", 1: ");
        Serial.print((send >> 8) & 0xFF);
        Serial.print(", 2: ");
        Serial.print((send >> 16) & 0xFF);
        Serial.print(", 3: ");
        Serial.println((send >> 24) & 0xFF);
        prev_command = command;
      }

      delay(100);

      if (millis() - heartbeat_tracker > heartbeat_interval_ms && command == 0){
        send_command(carCharacteristic, (int32_t)11);
        heartbeat_tracker = millis();
      }
  }
}

void send_command(BLECharacteristic& carCharacteristic, int32_t com){
    carCharacteristic.writeValue(com);
    if (debug == 3) Serial.println((int32_t)com);
    digitalWrite(LED_RED, HIGH);
    delay(10);
    digitalWrite(LED_RED, LOW);
}

void printBinary(int32_t value) {
    // Buffer to store the binary string with spaces
    char binaryString[40];  // "0000 0000" + null terminator
    int index = 0;

    // Iterate over each bit from MSB to LSB
    for (int i = 31; i >= 0; i--) {
        // Check if the i-th bit is set, then add '1' or '0' to the binary string
        binaryString[index++] = (value & (1UL << i)) ? '1' : '0';

        // Add a space after every 4 bits, except for the last one
        if (i % 4 == 0 && i != 0) {
            binaryString[index++] = ' ';
        }
    }
    binaryString[index] = '\0';  // Null-terminate the string

    Serial.println(binaryString); // Print the binary string to Serial
}
