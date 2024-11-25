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
#define EEPROM_SIZE       10

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
#include <EEPROM.h>

BluetoothSerial SerialBT;

TaskHandle_t Task1;
TaskHandle_t Task2;

int trayPosition [5]= { 0, 0, 0, 0, 0};

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

void processData(String data){
  if (data == "A"){ targetPosition = trayPosition[0]; moveUpStatus   = 1; }
  if (data == "B"){ targetPosition = trayPosition[1]; moveUpStatus   = 1; }
  if (data == "C"){ targetPosition = trayPosition[2]; moveUpStatus   = 1; }
  if (data == "D"){ targetPosition = trayPosition[3]; moveUpStatus   = 1; }
  if (data == "E"){ targetPosition = trayPosition[4]; moveUpStatus   = 1; }
  if (data == "Z"){ targetPosition = PICKUP_POSITION; moveDownStatus = 1; }
}

void processMasterData(String data){
  if (data.startsWith("*") && data.endsWith("#")){
    char traySelect = data.charAt(1);
    if (traySelect == 'A'){ targetPosition = trayPosition[0]; moveUpStatus   = 1; }
    if (traySelect == 'B'){ targetPosition = trayPosition[1]; moveUpStatus   = 1; }
    if (traySelect == 'C'){ targetPosition = trayPosition[2]; moveUpStatus   = 1; }
    if (traySelect == 'D'){ targetPosition = trayPosition[3]; moveUpStatus   = 1; }
    if (traySelect == 'E'){ targetPosition = trayPosition[4]; moveUpStatus   = 1; }
  } else if (data.startsWith("%") && data.endsWith("#")){
    char liftDown = data.charAt(1);
    if (liftDown == 'Z'){ targetPosition = PICKUP_POSITION; moveDownStatus = 1; }
  }
}             

void liftPostionSet(String data){
  char charTray = data.charAt(0);
  uint8_t tray = charTray - 'A';
  uint8_t pos  = data.substring(1, data.length()).toInt();
  if (DEBUG) {SerialBT.println("Tray: " + String(tray) + " Position: " + String(pos));}
  EEPROM.write(tray, pos);
  EEPROM.commit();
  trayPosition[tray] = pos*10;
  if (DEBUG) { SerialBT.println("Tray: " + String(tray) + "\t POS: " + String(trayPosition[tray]));}
}
void readSerialBT(){
  if (SerialBT.available()) {
    String incoming = SerialBT.readStringUntil('\n');
    incoming.trim();
    if (incoming.startsWith("L")){ liftPostionSet(incoming.substring(1, incoming.length()));}
    else { processData(incoming); }
    SerialBT.print(incoming);
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
    if (position >= HOMING_OFFSET) { analogWrite(MOTOR_UP_PIN, 0); homingStatus = true; }
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
    feedbackData = "";
    Serial2.println("*ERROR#");
    delay(100);
    analogWrite(MOTOR_UP_PIN, 0);
    moveUpStatus = 0;
    moveDownStatus = 0;
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
  } if (position >= targetPosition) {
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
    delay(500);
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

  trayPosition[0] = EEPROM.read(0)*10;
  trayPosition[1] = EEPROM.read(1)*10;
  trayPosition[2] = EEPROM.read(2)*10;
  trayPosition[3] = EEPROM.read(3)*10;
  trayPosition[4] = EEPROM.read(4)*10;
}


void coreZeroLoop() {
  readSerial2();
  readSerialBT();
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
  EEPROM.begin(EEPROM_SIZE);
  io_init();
  Serial.begin(9600);
  Serial2.begin(BAUDRATE);
  SerialBT.begin("VENDING_L1"); //Bluetooth device name
  dualCoreSetup();
  liftHoming();
}

void loop() {
  readEncoder();
  // debugIO();
}
