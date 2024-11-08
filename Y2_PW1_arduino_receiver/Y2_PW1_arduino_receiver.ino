#include <Servo.h>
#include <SoftwareSerial.h>
#include "configuration.h"

Servo head;
SoftwareSerial BLE(10, 11); // RX, TX pins for DFR0781 on UNO

unsigned long shutoff_period = 2000;
bool shutdown = false;
unsigned long heartbeat = 0;

int8_t drive_direction = 0;
int8_t turn_direction = 0;
uint8_t drive_power = 0;
uint8_t turn_power = 0;

int8_t last_drive_direction = 0;
int8_t last_turn_direction = 0;
uint8_t last_drive_power = 0;
uint8_t last_turn_power = 0;

bool do_print = false;
int32_t command = 0;
int32_t prev_command = 0;

int debug = 0;
uint32_t command_counter = 0;



void left(int16_t speed){
  //Serial.println((uint8_t)abs(speed));
  analogWrite(speedPinL, (uint8_t)abs(speed));
  if (speed >= 0){
    digitalWrite(dir1PinL, HIGH);
    digitalWrite(dir2PinL, LOW);
  } else {
    digitalWrite(dir1PinL, LOW);
    digitalWrite(dir2PinL, HIGH);
  }
}

void right(int16_t speed){
  //Serial.println((uint8_t)abs(speed));
  analogWrite(speedPinR, (uint8_t)abs(speed));
  if (speed >= 0){
    digitalWrite(dir1PinR, HIGH);
    digitalWrite(dir2PinR, LOW);
  } else {
    digitalWrite(dir1PinR, LOW);
    digitalWrite(dir2PinR, HIGH);
  }
}

void stop(){
  left(0);
  right(0);
}


/* Set motor speed */
void set_Motorspeed(uint8_t speed_L, uint8_t speed_R) {
  shutdown = false;
  analogWrite(speedPinL, speed_L); 
  analogWrite(speedPinR, speed_R);   
}

/* Serial (Bluetooth) control */
void do_Uart_Tick() {
  if (BLE.available()) {
    size_t len = BLE.available();
    uint8_t sbuf[len + 1];
    sbuf[len] = 0x00;
    BLE.readBytes(sbuf, len);
    memcpy(buffUART + buffUARTIndex, sbuf, len); // Store data
    buffUARTIndex += len;
    if (buffUARTIndex >= MAX_PACKETSIZE - 1) {
      buffUARTIndex = MAX_PACKETSIZE - 2;
    }
  }

  if (buffUARTIndex > 0) {
    buffUART[buffUARTIndex] = 0x00;
    
    if (debug == 1){
      Serial.print("0: ");
      Serial.print((uint32_t)buffUART[0] & 0xFF);
      Serial.print(", 1: ");
      Serial.print((uint32_t)buffUART[1] & 0xFF);
      Serial.print(", 2: ");
      Serial.print((uint32_t)buffUART[2] & 0xFF);
      Serial.print(", 3: ");
      Serial.println((uint32_t)buffUART[3] & 0xFF);
    }
    buffUARTIndex = 0;
    heartbeat = millis();

    command = (uint32_t)buffUART[0] & 0xFF;
    drive_power = (uint32_t)buffUART[1] & 0xFF;
    turn_power = (uint32_t)buffUART[2] & 0xFF;
    command_counter = (uint32_t)buffUART[3] & 0xFF;
  }

  if (!shutdown && ((millis() - heartbeat) > shutoff_period)){
    stop();
    //Serial.println("H STOP");
    shutdown = true;
  }
  
  // Handle control instructions
  // if (Uart_Date != 0) Serial.println(Uart_Date);
  if (command != prev_command){
    switch (command){
      case 0:                // nothing
        drive_direction = last_drive_direction;
        turn_direction = last_turn_direction;
        drive_power = last_drive_power;
        turn_power = last_turn_power;
      case 1:                // up
        drive_direction = 1;
        turn_direction = 0;
        break;
      case 2:                // up right
        drive_direction = 1;
        turn_direction = 1;
        break;
      case 3:                // up left
        drive_direction = 1;
        turn_direction = -1;
        break;
      case 4:                // down
        drive_direction = -1;
        turn_direction = 0;
        break;
      case 5:                // down right
        drive_direction = -1;
        turn_direction = 1;
        break;
      case 6:                // down left
        drive_direction = -1;
        turn_direction = -1;
        break;
      case 7:                // right
        drive_direction = 0;
        turn_direction = 1;
        break;
      case 8:                // left
        drive_direction = 0;
        turn_direction = -1;
        break;
      case 9:                // stop
        drive_direction = 0;
        turn_direction = 0;
        break;
      case 10:
        do_print = !do_print;
        Serial.println("Print toggled for clipping.");
        break;
    }
  }
  if (command != 0){
    prev_command = command;
  }
}

/* Car motor control */
int do_Drive_Tick() {
  if (turn_power == last_turn_power && turn_direction == last_turn_direction &&
      drive_power == last_drive_power && drive_direction == last_drive_direction){
    return;
  }
  if (drive_direction == 0 && turn_direction == 0){
    stop();
  } else {
    last_turn_power = turn_power;
    last_drive_power = drive_power;
    last_turn_direction = turn_direction;
    last_drive_direction = drive_direction;
    int left_speed = limit((drive_power * drive_direction) + (turn_power * turn_direction), -254, 254);
    int right_speed = limit((drive_power * drive_direction) - (turn_power * turn_direction), -254, 254);

    if (left_speed == 0 and right_speed == 0) return 0;

    if (do_print){
      Serial.print("Left: ");
      Serial.print(left_speed);
      Serial.print(", Right: ");
      Serial.print(right_speed);
      Serial.print(" || Drive: ");
      Serial.print(drive_power);
      Serial.print(", Turn: ");
      Serial.print(turn_power);
      Serial.print(", dir: ");
      Serial.print(drive_direction);
      Serial.print(", tur: ");
      Serial.println(turn_direction);
    } else {
      delay(10);
    }
    left(left_speed);
    right(right_speed);
  }
  return 1;
}

int limit(int inp, int lower, int upper){
  if (inp > upper){
    return upper;
  } else if (inp < lower){
    return lower;
  } else return inp;
}

void setup() {
  Serial.begin(9600); 
  BLE.begin(9600);         // Bluetooth connection
  Serial.println("Bluetooth ready for commands");

  pinMode(dir1PinL, OUTPUT); 
  pinMode(dir2PinL, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
  pinMode(dir1PinR, OUTPUT);
  pinMode(dir2PinR, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
  stop();

  digitalWrite(Trig_PIN, LOW);
  head.attach(SERVO_PIN);
  head.write(90);
}

void loop() {
  delay(40);
  do_Uart_Tick();
  do_Drive_Tick();
}