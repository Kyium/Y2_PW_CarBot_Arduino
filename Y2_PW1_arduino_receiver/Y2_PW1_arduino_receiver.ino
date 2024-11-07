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
bool do_print = true;
int32_t command = 0;
int32_t prev_command = 0;

int debug = 1;
uint32_t command_counter = 0;



void left(int8_t speed){
  analogWrite(speedPinL, abs(speed));
  if (speed >= 0){
    digitalWrite(dir1PinL, HIGH);
    digitalWrite(dir2PinL, LOW);
  } else {
    digitalWrite(dir1PinL, LOW);
    digitalWrite(dir2PinL, HIGH);
  }
}

void right(int8_t speed){
  analogWrite(speedPinR, abs(speed));
  if (speed >= 0){
    digitalWrite(dir1PinR, HIGH);
    digitalWrite(dir2PinR, LOW);
  } else {
    digitalWrite(dir1PinR, LOW);
    digitalWrite(dir2PinR, HIGH);
  }
}

void forward(int8_t speed){
  left(speed);
  right(speed);
}

void stop(){
  left(0);
  right(0);
}


/* Set motor speed */
void set_Motorspeed(int speed_L, int speed_R) {
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
      Serial.print((uint32_t)buffUART[0]);
      Serial.print(", 1: ");
      Serial.print((uint32_t)buffUART[1] & 0xFF);
      Serial.print(", 2: ");
      Serial.print((uint32_t)buffUART[2] & 0xFF);
      Serial.print(", 3: ");
      Serial.println((uint32_t)buffUART[3] & 0xFF);
    }
    buffUARTIndex = 0;
    heartbeat = millis();

    command = (uint32_t)buffUART[0];
    drive_power = (uint32_t)buffUART[1] & 0xFF;
    turn_power = (uint32_t)buffUART[2] & 0xFF;
    command_counter = (uint32_t)buffUART[3] & 0xFF;
  }

  if (!shutdown && ((millis() - heartbeat) > shutoff_period)){
    stop();
    Serial.println("H STOP");
    shutdown = true;
  }
  
  // Handle control instructions
  // if (Uart_Date != 0) Serial.println(Uart_Date);
  if (command != prev_command){
    switch (command){
      case 0: // nothing
        break;
      case 1:
        drive_direction = 1;
        turn_direction = 0;
        break;
      case 2:
        drive_direction = 1;
        turn_direction = 1;
        break;
      case 3:
        drive_direction = 1;
        turn_direction = -1;
        break;
      case 4:
        drive_direction = -1;
        turn_direction = 0;
        break;
      case 5:
        drive_direction = -1;
        turn_direction = 1;
        break;
      case 6:
        drive_direction = -1;
        turn_direction = -1;
        break;
      case 7:
        drive_direction = 0;
        turn_direction = 1;
        break;
      case 8:
        drive_direction = 0;
        turn_direction = -1;
        break;
      case 9:
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
void do_Drive_Tick() {
  if (drive_direction == 0 && turn_direction == 0){
    stop();
  } else {
    int left_speed = (drive_power * drive_direction) + (turn_power * turn_direction);
    int right_speed = (drive_power * drive_direction) - (turn_power * turn_direction);
    if (do_print){
      Serial.print("Left: ");
      Serial.print(left_speed);
      Serial.print(", Left clipped: ");
      Serial.print(limit(left_speed, -255, 255));
      Serial.print(", Right: ");
      Serial.print(right_speed);
      Serial.print(", Right Clipped: ");
      Serial.print(limit(right_speed, -255, 255));
      Serial.print(" || Drive: ");
      Serial.print(drive_power);
      Serial.print(", Turn: ");
      Serial.print(turn_power);
      Serial.print(", dir: ");
      Serial.print(drive_direction);
      Serial.print(", tur: ");
      Serial.println(turn_direction);
    }
  left(left_speed);
  right(right_speed);
  }
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
  do_Uart_Tick();
  do_Drive_Tick();
}