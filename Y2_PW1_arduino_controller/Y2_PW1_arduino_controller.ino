#include <ArduinoBLE.h>

#define pin_x_analogue A1
#define pin_y_analogue A0
#define pin_d_digital 2

int x = 0;
int y = 1;
int d = 0;
int debug = 3;
int8_t drive_power = 0;
int8_t turn_power = 0;
int8_t motor_power = 0;
int8_t prev_motor_power = 0;
char drive_direction = '0';
char turn_direction = '0';
char prev_com = '0';
unsigned long heartbeat_interval_ms = 750;
unsigned long heartbeat_tracker = 0;
unsigned long press_delay = 0;

int8_t command = 0;
int32_t send = 0;


void setup() {
  pinMode(pin_x_analogue, INPUT);
  pinMode(pin_y_analogue, INPUT);
  pinMode(pin_d_digital, INPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  Serial.begin(9600);
  delay(1000);
  //while (!Serial);
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
    BLE.scanForUuid("d6eb");
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
    //Serial.println("Ready.");
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

      drive_power = map(abs(y - 2048) / 16, 0, 127, 10, 126);
      turn_power = map(abs(x - 2048) / 16, 0, 128, 244, 128);

      //Serial.print("Drive: ");
      //Serial.print(drive_power);
      //Serial.print(", Turn: ");
      //Serial.println(turn_power);

      if (d && (millis() - press_delay > 1000)){
        send_command_if_non_consecutive(carCharacteristic, 8);
        press_delay = 1000;
      }

      if (y > 2400){
        drive_direction = 1;
        send_command_if_non_consecutive(carCharacteristic, 1);
      } else if (y < 1800) {
        drive_direction = 2;
        send_command_if_non_consecutive(carCharacteristic, 2);
      } else {
        drive_direction = 0;
      }
      if (x > 2400) {
        turn_direction = 4;
        send_command_if_non_consecutive(carCharacteristic, 4);
      } else if (x < 1800) {
        turn_direction = 5;
        send_command_if_non_consecutive(carCharacteristic, 5);
      } else {
        turn_direction = 0;
      }

      if (drive_direction == 0 && turn_direction == 0){
        send_command_if_non_consecutive(carCharacteristic, 6);
      } else if (drive_direction == 0){
        send_command_if_non_consecutive(carCharacteristic, 7);
      }else if (turn_direction == 0){
        send_command_if_non_consecutive(carCharacteristic, 3);
      }


      if (drive_power > 25){
        send_command(carCharacteristic, (char)drive_power);
        delay(10);
      }
      if (turn_power < -25){
        send_command(carCharacteristic, (char)turn_power);
        delay(10);
      }

      if (millis() - heartbeat_tracker > heartbeat_interval_ms){
        send_command(carCharacteristic, (char)9);
        heartbeat_tracker = millis();
      }
  }
}

void send_command_if_non_consecutive(BLECharacteristic& carCharacteristic ,char com){
  if (com != prev_com){
    carCharacteristic.writeValue(com);
    if (debug == 3) Serial.println((int8_t)com);
    prev_com = com;
    digitalWrite(LED_RED, HIGH);
    delay(10);
    digitalWrite(LED_RED, LOW);
  }
}

void send_command(BLECharacteristic& carCharacteristic ,char com){
    carCharacteristic.writeValue(com);
    if (debug == 3) Serial.println((int8_t)com);
    digitalWrite(LED_RED, HIGH);
    delay(10);
    digitalWrite(LED_RED, LOW);
}


