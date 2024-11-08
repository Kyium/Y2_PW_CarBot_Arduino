#include <Servo.h>
#include <SoftwareSerial.h>
#include "configuration.h"

Servo head;

const int bufferSize = 32;
char inputBuffer[bufferSize];
int bufferIndex = 0;

void left(int16_t speed){
  Serial.println((uint8_t)abs(speed));
  analogWrite(speedPinL, (uint8_t)abs(speed));
  if (speed > 0){
    digitalWrite(dir1PinL, HIGH);
    digitalWrite(dir2PinL, LOW);
  } else if (speed < 0){
    digitalWrite(dir1PinL, LOW);
    digitalWrite(dir2PinL, HIGH);
  } else {
    digitalWrite(dir1PinL, LOW);
    digitalWrite(dir2PinL, LOW);
  }
}

void right(int16_t speed){
  Serial.println((uint8_t)abs(speed));
  analogWrite(speedPinR, (uint8_t)abs(speed));
  if (speed > 0){
    digitalWrite(dir1PinR, HIGH);
    digitalWrite(dir2PinR, LOW);
  } else if(speed < 0){
    digitalWrite(dir1PinR, LOW);
    digitalWrite(dir2PinR, HIGH);
  } else {
    digitalWrite(dir1PinR, LOW);
    digitalWrite(dir2PinR, LOW);
  }
}

void stop(){
  left(0);
  right(0);
}


/* Set motor speed */
void set_Motorspeed(uint8_t speed_L, uint8_t speed_R) {
  analogWrite(speedPinL, speed_L); 
  analogWrite(speedPinR, speed_R);   
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
  Serial.println("ready for commands");

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
    // Check if there's any serial data available
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    
    // If newline character, process command
    if (receivedChar == '\n') {
      inputBuffer[bufferIndex] = '\0'; // Null-terminate the string
      processCommand(inputBuffer); // Process the command
      bufferIndex = 0; // Reset buffer index for the next command
    } 
    // If buffer not full, store character in buffer
    else if (bufferIndex < bufferSize - 1) {
      inputBuffer[bufferIndex++] = receivedChar;
    }
  }
}

void processCommand(char* command) {
  // Find the position of the colon
  char* separator = strchr(command, ':');
  
  // If colon found, split the command and process
  if (separator != nullptr) {
    *separator = '\0'; // Split command at ':'
    char* key = command;           // Command name (e.g., "left" or "right")
    char* valueStr = separator + 1; // Command value (e.g., "0" or "255")

    int value = atoi(valueStr); // Convert value to an integer

    // Process the command based on key and value
    if (strcmp(key, "left") == 0) {
      Serial.print("Left command received with value: ");
      Serial.println(value);
      right(value);
      // Handle left command with the value
    } 
    else if (strcmp(key, "right") == 0) {
      Serial.print("Right command received with value: ");
      Serial.println(value);
      left(value);
      // Handle right command with the value
    } 
    else {
      Serial.print("Unknown command: ");
      Serial.println(key);
    }
  } else {
    Serial.println("Invalid command format.");
  }
}