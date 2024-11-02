#define DEBUG 1

#define BAUDRATE 115200

#define HOMING_SPEED  50
#define MOVING_SPEED  50
#define HOMING_OFFSET 45

#define MOTOR_DOWN_PIN    23
#define MOTOR_UP_PIN      22

#define ENCODER_L_PIN 18
#define ENCODER_R_PIN 19

#define LIMIT_UP      25
#define LIMIT_DOWN    33

#define SENSITIVITY 16

#define DELAY_TIME_IN_ms 1

#define COMMAND_FOR_LEFT  Serial.println("L")
#define COMMAND_FOR_RIGHT Serial.println("R")

#define UP_LIMIT_DETECT   digitalRead(LIMIT_UP)   == 0
#define DOWN_LIMIT_DETECT digitalRead(LIMIT_DOWN) == 0

#define TRAY_1_POSITION 1030
#define TRAY_2_POSITION 850
#define TRAY_3_POSITION 670
#define TRAY_4_POSITION 530
#define TRAY_5_POSITION 390
#define TRAY_6_POSITION 250

#define PICKUP_POSITION 0


#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

bool homingStatus = false;

bool currentState;
bool lastState;

uint32_t position = 0;

uint32_t encoder_l_counter = 0;
uint32_t encoder_r_counter = 0;

uint32_t targetPosition = 0;

uint8_t movingSpeed = 10;

void read_encoder() { 
  currentState = digitalRead(ENCODER_L_PIN);
  if (currentState != lastState)
  {     
    if (digitalRead(ENCODER_R_PIN) != currentState) { encoder_l_counter++; encoder_r_counter =0; if(encoder_l_counter > SENSITIVITY){ COMMAND_FOR_LEFT; encoder_l_counter=0; position--; Serial.println(position);} } 
    else {encoder_r_counter ++; encoder_l_counter =0; if(encoder_r_counter > SENSITIVITY) { COMMAND_FOR_RIGHT; encoder_r_counter =0; position++; Serial.println(position);} }
  }
  lastState = currentState; 
}


void readSerial2() {
  if (Serial2.available()) {
    String incoming = Serial2.readString();
    incoming.trim();
    if (DEBUG) { Serial.println(incoming); SerialBT.println(incoming); }   
  }
}


void debugIO() {
  Serial.println("UP: " + String(digitalRead(LIMIT_UP)) + " DOWN: " + String(digitalRead(LIMIT_DOWN)));
}

void moveMotor(){
  if (position < targetPosition) {
    for (int i = 10; i < MOVING_SPEED; i++) {
      movingSpeed = i;
      analogWrite(MOTOR_UP_PIN, movingSpeed);
      analogWrite(MOTOR_DOWN_PIN, 0); 
      }
    analogWrite(MOTOR_UP_PIN, movingSpeed);
    analogWrite(MOTOR_DOWN_PIN, 0);
    while (position != targetPosition) { read_encoder(); }
    analogWrite(MOTOR_UP_PIN, 0);
    delay(1000);
    targetPosition = PICKUP_POSITION;
  } if (position > targetPosition) {
    analogWrite(MOTOR_DOWN_PIN, MOVING_SPEED);
    analogWrite(MOTOR_UP_PIN, 0);
    while (position != targetPosition) { read_encoder(); }
    analogWrite(MOTOR_DOWN_PIN, 0);
    delay(1000);
  } else {
    analogWrite(MOTOR_DOWN_PIN, 0);
    analogWrite(MOTOR_UP_PIN, 0);
  }

}

void processData(char data){
  if (data == 'A'){ targetPosition = TRAY_1_POSITION; }
  if (data == 'B'){ targetPosition = TRAY_2_POSITION; }
  if (data == 'C'){ targetPosition = TRAY_3_POSITION; }
  if (data == 'D'){ targetPosition = TRAY_4_POSITION; }
  if (data == 'E'){ targetPosition = TRAY_5_POSITION; }
  if (data == 'F'){ targetPosition = TRAY_6_POSITION; }
}

void readSerialBT(){
  if (SerialBT.available()) {
    char incoming = SerialBT.read();
    processData(incoming);  
  }
}

void liftHoming() {
  analogWrite(MOTOR_DOWN_PIN, HOMING_SPEED);
  analogWrite(MOTOR_UP_PIN,   0);
  if (DEBUG){ Serial.println("Homing...."); }
  while (digitalRead(LIMIT_DOWN) == 1) { }
  analogWrite(MOTOR_DOWN_PIN, 0);
  analogWrite(MOTOR_DOWN_PIN, 0);
  position = 0;
  if (DEBUG){ Serial.println("Homing Completed!!"); }
  delay(1000);
  analogWrite(MOTOR_DOWN_PIN, 0);
  analogWrite(MOTOR_UP_PIN,   HOMING_SPEED);
  if (DEBUG){ Serial.println("Setting Offset...."); }
  while (!homingStatus) {
    read_encoder();
    if (position >= HOMING_OFFSET) { homingStatus = true; }
  }
  analogWrite(MOTOR_UP_PIN, 0);
  if (DEBUG) {Serial.println("Offset Setting Completed!!");}
  delay(1000);
  position = 0;
}

void io_init(){
  pinMode(ENCODER_L_PIN,INPUT_PULLUP);
  pinMode(ENCODER_R_PIN,INPUT_PULLUP);

  pinMode(LIMIT_UP,   INPUT_PULLUP);
  pinMode(LIMIT_DOWN, INPUT_PULLUP);

  // pinMode(MOTOR_DOWN_PIN, OUTPUT);
  // pinMode(MOTOR_UP_PIN,   OUTPUT);
  analogWrite(MOTOR_DOWN_PIN, 0);
  analogWrite(MOTOR_UP_PIN,   0);
}

void setup() {
  io_init();
  Serial.begin(9600);
  Serial2.begin(BAUDRATE);
  SerialBT.begin("VENDING_MACHINE"); //Bluetooth device name
  liftHoming();
}

void loop() {
  readSerialBT();
  read_encoder();
  moveMotor();
  // debugIO();
}
