#define DEBUG             1       // DON'T CHANGE

#define BAUDRATE          115200

#define HOMING_SPEED      30
#define MOVING_DOWN_SPEED 80
#define MAX_MOVING_SPEED  100
#define MIN_MOVING_SPEED  30
#define HOMING_OFFSET     100

#define ACCL_END_POS      255
#define DECL_END_POS      255

#define MOTOR_DOWN_PIN    23
#define MOTOR_UP_PIN      22

#define ENCODER_L_PIN     18
#define ENCODER_R_PIN     19

#define LIMIT_UP          25
#define LIMIT_DOWN        33

#define SENSITIVITY       1

#define DELAY_TIME_IN_ms  1

#define COMMAND_FOR_LEFT  Serial.println("L")
#define COMMAND_FOR_RIGHT Serial.println("R")

#define UP_LIMIT_DETECT   digitalRead(LIMIT_UP)   == 0
#define DOWN_LIMIT_DETECT digitalRead(LIMIT_DOWN) == 0

#define TRAY_1_POSITION   1635
#define TRAY_2_POSITION   1415
#define TRAY_3_POSITION   1205
#define TRAY_4_POSITION   1015
#define TRAY_5_POSITION   810
#define TRAY_6_POSITION   600

#define PICKUP_POSITION   0


#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

TaskHandle_t Task1;
TaskHandle_t Task2;

bool  currentState;
bool  lastState;

bool  homingStatus     = 0;
bool  moveUpStatus     = 0;
bool  moveDownStatus   = 0;

uint32_t  position     = 0;

uint32_t  encoder_l_counter  = 0;
uint32_t  encoder_r_counter  = 0;

uint32_t  targetPosition     = 0;
uint8_t   movingSpeed        = 0;

String    feedbackData = "";

void readEncoder() { 
  currentState = digitalRead(ENCODER_L_PIN);
  if (currentState != lastState){     
    if (digitalRead(ENCODER_R_PIN) != currentState){ 
      encoder_l_counter++; encoder_r_counter =0; 
      if (encoder_l_counter > SENSITIVITY){ 
        encoder_l_counter=0; position--; 
        if (DEBUG){ Serial.println(position);}
      } 
    } else {
      encoder_r_counter++; encoder_l_counter = 0; 
      if(encoder_r_counter > SENSITIVITY) { 
        encoder_r_counter =0; position++; 
        if (DEBUG) {Serial.println(position);} 
      }
    }
  }
  lastState = currentState; 
}

void debugIO() {
  Serial.println("UP: " + String(digitalRead(LIMIT_UP)) + " DOWN: " + String(digitalRead(LIMIT_DOWN)));
}

void processData(char data){
  if (data == 'A'){ targetPosition = TRAY_1_POSITION; moveUpStatus   = 1; }
  if (data == 'B'){ targetPosition = TRAY_2_POSITION; moveUpStatus   = 1; }
  if (data == 'C'){ targetPosition = TRAY_3_POSITION; moveUpStatus   = 1; }
  if (data == 'D'){ targetPosition = TRAY_4_POSITION; moveUpStatus   = 1; }
  if (data == 'E'){ targetPosition = TRAY_5_POSITION; moveUpStatus   = 1; }
  if (data == 'F'){ targetPosition = TRAY_6_POSITION; moveUpStatus   = 1; }
  if (data == 'Z'){ targetPosition = PICKUP_POSITION; moveDownStatus = 1; }
}

void processMasterData(String data){
  if (data.startsWith("*") && data.endsWith("#")){
    char traySelect = data.charAt(1);
    if (traySelect == 'A'){ targetPosition = TRAY_1_POSITION; moveUpStatus   = 1; }
    if (traySelect == 'B'){ targetPosition = TRAY_2_POSITION; moveUpStatus   = 1; }
    if (traySelect == 'C'){ targetPosition = TRAY_3_POSITION; moveUpStatus   = 1; }
    if (traySelect == 'D'){ targetPosition = TRAY_4_POSITION; moveUpStatus   = 1; }
    if (traySelect == 'E'){ targetPosition = TRAY_5_POSITION; moveUpStatus   = 1; }
    if (traySelect == 'F'){ targetPosition = TRAY_6_POSITION; moveUpStatus   = 1; }
  } else if (data.startsWith("%") && data.endsWith("#")){
    char liftDown = data.charAt(1);
    if (liftDown == 'Z'){ targetPosition = PICKUP_POSITION; moveDownStatus = 1; }
  }
}             
void readSerialBT(){
  if (SerialBT.available()) {
    char incoming = SerialBT.read();
    SerialBT.print(incoming);
    processData(incoming);  
  }
}

void readSerial2() {
  if (Serial2.available()) {
    String incoming = Serial2.readString();
    incoming.trim();
    feedbackData = incoming;
    processMasterData(incoming);
    if (DEBUG) { Serial.println(incoming); SerialBT.println(incoming); }   
  }
}

void liftHoming() {
  analogWrite(MOTOR_DOWN_PIN, HOMING_SPEED);
  analogWrite(MOTOR_UP_PIN,   0);
  
  if (DEBUG){ Serial.println("Homing...."); }
  while (digitalRead(LIMIT_DOWN) == 1) { yield(); }  // Wait for the limit switch trigger
  analogWrite(MOTOR_DOWN_PIN, 0);
  delay(1000);
  position = 0;
  if (DEBUG){ Serial.println("Homing Completed!!"); }
  if (DEBUG){ Serial.println("Setting Offset...."); }
  analogWrite(MOTOR_UP_PIN,   HOMING_SPEED);
  while (!homingStatus) {
    readEncoder();
    if (position == HOMING_OFFSET) { analogWrite(MOTOR_UP_PIN, 0); homingStatus = true; }
    yield();
  }
  analogWrite(MOTOR_UP_PIN, 0);
  delay(1000);
  if (DEBUG) {Serial.println("Offset Setting Completed!!");}
  position = 0;
}

void limitCheck () {
  if (position > TRAY_1_POSITION + 1000){ analogWrite(MOTOR_DOWN_PIN, 0); analogWrite(MOTOR_UP_PIN, 0); position = 0; } 
  if (UP_LIMIT_DETECT) {
    analogWrite(MOTOR_UP_PIN, 0);
    homingStatus = 0;
    movingSpeed = 0;
    liftHoming();
  }
  if (DOWN_LIMIT_DETECT) {
    analogWrite(MOTOR_DOWN_PIN, 0);
    analogWrite(MOTOR_UP_PIN, 0);
    homingStatus = 0;
    liftHoming();
  }
}

void moveUp() {
  if (position < targetPosition && !UP_LIMIT_DETECT) {
    if (position < 255) {
      analogWrite(MOTOR_UP_PIN, movingSpeed);
      analogWrite(MOTOR_DOWN_PIN, 0);
      if (movingSpeed < MAX_MOVING_SPEED) { movingSpeed++; }
      delay(3);
    } else if (targetPosition - position < 250) {
      analogWrite(MOTOR_UP_PIN, movingSpeed);
      analogWrite(MOTOR_DOWN_PIN, 0);
      if (movingSpeed > HOMING_SPEED) { movingSpeed--; }
    }
  } if (position == targetPosition || UP_LIMIT_DETECT) {
    analogWrite(MOTOR_UP_PIN, 0);
    analogWrite(MOTOR_DOWN_PIN, 0);
    moveUpStatus = 0;
    movingSpeed = 0;
    Serial2.println(feedbackData);
    feedbackData = "";
  }
}

void moveDown() {
  analogWrite(MOTOR_DOWN_PIN, movingSpeed);
  analogWrite(MOTOR_UP_PIN, 0);
  if (movingSpeed < MOVING_DOWN_SPEED) { movingSpeed++; }
  delay(3);
  if (DOWN_LIMIT_DETECT) {
    analogWrite(MOTOR_DOWN_PIN, 0);
    analogWrite(MOTOR_UP_PIN, 0);
    delay(100);
    movingSpeed = 0;
    moveDownStatus = 0;
    homingStatus = 0;
    liftHoming();
    delay(1000);
    Serial2.println("$X#");
    position = 0;
  }
}

void io_init(){
  pinMode(ENCODER_L_PIN,INPUT_PULLUP);
  pinMode(ENCODER_R_PIN,INPUT_PULLUP);

  pinMode(LIMIT_UP,   INPUT_PULLUP);
  pinMode(LIMIT_DOWN, INPUT_PULLUP);

  analogWrite(MOTOR_DOWN_PIN, 0);
  analogWrite(MOTOR_UP_PIN,   0);
}


void coreZeroLoop() {
  if (moveUpStatus)   { moveUp();   }
  if (moveDownStatus) { moveDown(); }
  limitCheck();
  delay(1);
  yield();
}

void Task1code( void * pvParameters ){ for(;;){ coreZeroLoop(); yield();} }
void Task2code( void * pvParameters ){ for(;;){ yield(); } }

void dualCoreSetup()
{
  delay(500);
  xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 1, &Task1, 0);                       
  delay(500); 
  xTaskCreatePinnedToCore(Task2code, "Task2", 10000, NULL, 1, &Task2, 1);      
  delay(500); 
}

void setup() {
  io_init();
  Serial.begin(9600);
  Serial2.begin(BAUDRATE);
  SerialBT.begin("VENDING_L1_TEST"); //Bluetooth device name
  dualCoreSetup();
  liftHoming();
}

void loop() {
  readSerial2();
  readSerialBT();
  readEncoder();
  // debugIO();
}
