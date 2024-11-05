#include <Servo.h>
#include <SoftwareSerial.h>
#include "configuration.h"

Servo head;
SoftwareSerial BLE(10, 11); // RX, TX pins for DFR0781 on UNO

int Drive_Power = 255;
int Prev_Drive_Power = 255;
unsigned long shutoff_period = 2000;
bool shutdown = false;
unsigned long heartbeat = 0;
char prev_Uart_date = '0';
bool non_com_recv = false;


DN Prev_Drive_Num = DEF;

/* Motor control */
void go_Advance() {
  digitalWrite(dir1PinL, HIGH);
  digitalWrite(dir2PinL, LOW);
  digitalWrite(dir1PinR, HIGH);
  digitalWrite(dir2PinR, LOW);
}
void go_Left() {
  digitalWrite(dir1PinL, HIGH);
  digitalWrite(dir2PinL, LOW);
  digitalWrite(dir1PinR, LOW);
  digitalWrite(dir2PinR, HIGH);
}
void go_Right() {
  digitalWrite(dir1PinL, LOW);
  digitalWrite(dir2PinL, HIGH);
  digitalWrite(dir1PinR, HIGH);
  digitalWrite(dir2PinR, LOW);
}
void go_Back() {
  digitalWrite(dir1PinL, LOW);
  digitalWrite(dir2PinL, HIGH);
  digitalWrite(dir1PinR, LOW);
  digitalWrite(dir2PinR, HIGH);
}
void stop_Stop() {
  digitalWrite(dir1PinL, LOW);
  digitalWrite(dir2PinL, LOW);
  digitalWrite(dir1PinR, LOW);
  digitalWrite(dir2PinR, LOW);
}

/* Set motor speed */
void set_Motorspeed(int speed_L, int speed_R) {
  shutdown = false;
  analogWrite(speedPinL, speed_L); 
  analogWrite(speedPinR, speed_R);   
}

/* Buzzer control */
void buzz_ON() {
  digitalWrite(BUZZ_PIN, LOW);
}
void buzz_OFF() {
  digitalWrite(BUZZ_PIN, HIGH);
}
void alarm() {
  buzz_ON();
  delay(100);
  buzz_OFF();
}

/* Serial (Bluetooth) control */
void do_Uart_Tick() {
  char Uart_Date = 0;
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
    }
    buffUARTIndex = 0;
    heartbeat = millis();
  }

  if (!shutdown && ((millis() - heartbeat) > shutoff_period)){
    stop_Stop(); 
    Serial.println("H STOP");
    shutdown = true;
  }
  
  // Handle control instructions
  switch (Uart_Date) {
    case '0': non_com_recv = false; Serial.println("HB"); break; // heartbeat
    case '1': non_com_recv = false; break;
    case '2': non_com_recv = false; Drive_Num = GO_ADVANCE; BLE.println("forward"); break;
    case '3': non_com_recv = false; break;
    case '4': non_com_recv = false; Drive_Num = GO_LEFT; BLE.println("turn left"); break;
    case '5': non_com_recv = false; Drive_Num = STOP_STOP; buzz_OFF(); BLE.println("stop"); break;
    case '6': non_com_recv = false; Drive_Num = GO_RIGHT; BLE.println("turn right"); break;
    case '7': non_com_recv = false; break;
    case '8': non_com_recv = false; Drive_Num = GO_BACK; BLE.println("go back"); break;
    case '9': non_com_recv = false; break;
    default: non_com_recv = true; break;
  }
  if ((prev_Uart_date != Uart_Date) && ((int)Uart_Date > 0) && non_com_recv){
    Drive_Power = ((int)Uart_Date) * 2;
    Serial.println(Drive_Power);
    prev_Uart_date = Uart_Date;
  }
}

/* Car motor control */
void do_Drive_Tick() {
  if ((Drive_Num != Prev_Drive_Num) || (Drive_Power != Prev_Drive_Power)){
    switch (Drive_Num) {
      case GO_ADVANCE: 
        go_Advance(); 
        set_Motorspeed(Drive_Power, Drive_Power); 
        Serial.print("GO forward: "); 
        Serial.println(Drive_Power);
        Prev_Drive_Num = Drive_Num;
        Prev_Drive_Power = Drive_Power;
        break;
      case GO_LEFT: 
        go_Left(); 
        set_Motorspeed(Drive_Power, Drive_Power); 
        Serial.print("GO left: ");
        Serial.println(Drive_Power); 
        Prev_Drive_Num = Drive_Num;
        Prev_Drive_Power = Drive_Power;
        break;
      case GO_RIGHT: 
        go_Right(); 
        set_Motorspeed(Drive_Power, Drive_Power); 
        Serial.print("GO right: ");
        Serial.println(Drive_Power); 
        Prev_Drive_Num = Drive_Num;
        Prev_Drive_Power = Drive_Power;
        break;
      case GO_BACK: 
        go_Back(); 
        set_Motorspeed(Drive_Power, Drive_Power); 
        Serial.print("GO backward: ");
        Serial.println(Drive_Power); 
        Prev_Drive_Num = Drive_Num;
        Prev_Drive_Power = Drive_Power;
        break;
      case STOP_STOP: 
        stop_Stop(); 
        Serial.println("C STOP");
        Prev_Drive_Num = Drive_Num;
        Prev_Drive_Power = Drive_Power;
        break;
      default: break;
    }
  }
}

void setup() {
  Serial.begin(9600);      // For Serial Monitor
  BLE.begin(9600);         // Bluetooth connection
  Serial.println("Bluetooth ready for commands");

  pinMode(dir1PinL, OUTPUT); 
  pinMode(dir2PinL, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
  pinMode(dir1PinR, OUTPUT);
  pinMode(dir2PinR, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
  stop_Stop();

  pinMode(Trig_PIN, OUTPUT); 
  pinMode(Echo_PIN, INPUT); 

  pinMode(BUZZ_PIN, OUTPUT);
  buzz_OFF();  

  pinMode(LFSensor_0, INPUT);
  pinMode(LFSensor_1, INPUT);
  pinMode(LFSensor_2, INPUT);
  pinMode(LFSensor_3, INPUT);
  pinMode(LFSensor_4, INPUT); 

  digitalWrite(Trig_PIN, LOW);
  head.attach(SERVO_PIN);
  head.write(90);
}

void loop() {
  do_Uart_Tick();
  do_Drive_Tick();
}