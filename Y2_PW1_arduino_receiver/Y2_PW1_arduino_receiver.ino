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
int8_t drive_power = 0;
int8_t turn_power = 0;
bool do_print = true;
int8_t last_signal = 0;
bool changed_signal = false;
char Uart_Date = 0;


void left(int8_t speed){
  analogWrite(speedPinL, speed);
  if (speed >= 0){
    digitalWrite(dir1PinL, HIGH);
    digitalWrite(dir2PinL, LOW);
  } else {
    digitalWrite(dir1PinL, LOW);
    digitalWrite(dir2PinL, HIGH);
  }
}

void right(int8_t speed){
  analogWrite(speedPinR, speed);
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
  if ((int8_t)Uart_Date != 0) last_signal = (int8_t)Uart_Date;
  Uart_Date = 0;
  if (BLE.available()) {
    size_t len = BLE.available();
    uint8_t sbuf[len + 1];
    sbuf[len] = 0x00;
    BLE.readBytes(sbuf, len);
    memcpy(buffUART + buffUARTIndex, sbuf, len); // Store data
    buffUARTIndex += len;
    preUARTTick = millis();
    if (buffUARTIndex >= MAX_PACKETSIZE - 1) {
      buffUARTIndex = MAX_PACKETSIZE - 2;
      preUARTTick -= 200;
    }
  }

  if (buffUARTIndex > 0 && (millis() - preUARTTick >= 10)) {
    buffUART[buffUARTIndex] = 0x00;
    if (buffUART[0] == 'C') {
      BLE.println(buffUART);
      BLE.println("Parameters modified!");
      sscanf(buffUART, "CMD%d,%d,%d", &distancelimit, &sidedistancelimit, &turntime);
    } else {
      
      Uart_Date = buffUART[0];
      changed_signal = last_signal != (int8_t)Uart_Date;
    }
    buffUARTIndex = 0;
    heartbeat = millis();
  }

  if (!shutdown && ((millis() - heartbeat) > shutoff_period)){
    stop();
    Serial.println("H STOP");
    shutdown = true;
  }
  
  // Handle control instructions
  if ((int)Uart_Date != 0) Serial.println((int) Uart_Date);

  switch ((int8_t)Uart_Date){
    case 0: // nothing
      break;
    case 1:
      drive_direction = 1;
      break;
    case 2:
      drive_direction = -1;
      break;
    case 3:
      turn_direction = 0;
      turn_power = 0;
      break;
    case 4:
      turn_direction = 1;
      break;
    case 5:
      turn_direction = -1;
      break;
    case 6:
      drive_direction = 0;
      turn_direction = 0;
      drive_power = 0;
      turn_power = 0;
      break;
    case 7:
      drive_direction = 0;
      drive_power = 0;
      break;
    case 8:
      do_print = !do_print;
      break;
    case 9: // heartbeat
      break;
  }
  if ((int)Uart_Date > 9){
    drive_power = (int8_t)Uart_Date;
    Serial.print("Drive Power: ");
    Serial.println(drive_power);
  }
  if ((int)Uart_Date < -9){
    turn_power = abs((int8_t)Uart_Date);
    Serial.print("Turn Power: ");
    Serial.println(turn_power);
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
      Serial.println(turn_power);
    }
  }
  //right(left_speed);
  //right(right_speed);
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
  if (changed_signal) do_Drive_Tick();
}