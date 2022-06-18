#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>
#include "HX711.h"
#define RST_PIN 22   
#define SS_PIN  21
String ctrl;
char motor;
int subject = 1;
int steerVal = 0;
const int LOADCELL_DOUT_PIN = 33;
const int LOADCELL_SCK_PIN = 32;
const int motor1Pin1 = 25; 
const int motor1Pin2 = 26; 
const int enable1Pin = 15;
float calibration_factor = -266345;
int pos = 0; //Servo Position
const int servoPin = 14;
const int trigPin1 = 5;
const int echoPin1 = 27;
const int trigPin2 = 4;
const int echoPin2 = 2;
const int trigPin3 = 13;
const int echoPin3 = 12;
long duration, distance, RightSensor, MiddleSensor, LeftSensor;
MFRC522 mfrc522(SS_PIN, RST_PIN); // Create a MFRC522 instance
MFRC522::MIFARE_Key key;          // Prepare key - all keys are set to FFFFFFFFFFFFh at chip delivery from the factory
HX711 scale;
Servo myservo;
//****************************************************************************************
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#define bleServerName "iTrolley"
std::string weight ="";
std::string item="";
bool deviceConnected = false;
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"
#define CHARACTERISTIC_UUID_RFID "b2bef01d-51f6-4ad1-8ddf-94bf5e1ce06a"
#define CHARACTERISTIC_UUID_LOAD "b2bef02d-51f6-4ad1-8ddf-94bf5e1ce06a"
BLEServer *pServer;
BLEService *pService;
BLECharacteristic *bleTotalWeightCharacteristic;
BLECharacteristic *bleRFIDidValueCharacteristic;
//****************************************************************************************
class MyServerCallbacks: public BLEServerCallbacks { 
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    pServer->getAdvertising()->start();
  }
};
//****************************************************************************************
void setup() {
  Serial.begin(115200);  
  SPI.begin();
  mfrc522.PCD_Init();  // Init MFRC522 card
  for (byte i = 0; i < 6; i++) { 
    key.keyByte[i] = 0xFF;  // Prepare the key
  }
  dump_byte_array(key.keyByte, MFRC522::MF_KEY_SIZE);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN); //Load Cell Init
  scale.set_scale();
  scale.tare();     //Reset the scale to 0
  long zero_factor = scale.read_average(); //Get a baseline reading //Serial.print("Zero factor: "); Serial.println(zero_factor);
  pinMode(motor1Pin1, OUTPUT); // Motor Pins
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  ESP32PWM::allocateTimer(0); // Allow allocation of all timers for Servo Operation
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50); // standard 50 hz servo
  myservo.attach(servoPin, 500, 2500);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  BLEDevice::init(bleServerName);
  pServer = BLEDevice::createServer();
  pService = pServer->createService(SERVICE_UUID);
  bleTotalWeightCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_LOAD,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE);
  bleRFIDidValueCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RFID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE);
  bleTotalWeightCharacteristic->addDescriptor(new BLE2902());
  bleRFIDidValueCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pServer->setCallbacks(new MyServerCallbacks());  //Server Callback Handler  
  bleTotalWeightCharacteristic->setValue("");
  bleRFIDidValueCharacteristic->setValue("");
  BLEAdvertising *pAdvertising = pServer->getAdvertising();   
  pAdvertising->setScanResponse(true);           
  pAdvertising->addServiceUUID(SERVICE_UUID);  
  pServer->getAdvertising()->start();
  Serial.println("BLE Advertising Now!");
}
//****************************************************************************************
void loop() {
  getLoadReading();
  byte block;
  byte len;
  //-------------------------------------------------------------------------------------------------------------------
  if (mfrc522.PICC_IsNewCardPresent()) {  // Reset the loop if no new card present on the sensor/reader.
  if (!mfrc522.PICC_ReadCardSerial())  {  // Select one of the cards
    return; }
  Serial.println(); Serial.println(); Serial.println("|Card Detected:");  //mfrc522.PICC_DumpDetailsToSerial(&(mfrc522.uid)); //dump some details about the card
  MFRC522::StatusCode status; //mfrc522.PICC_DumpToSerial(&(mfrc522.uid));  //Memory Dump_all blocks in hex
  byte sector         = 0; //Second Sector 
  byte blockAddr      = 1; //block 0 in sector 0-Manufacturer Data
  byte trailerBlock   = 3;
  byte buffer[18];         //to store read data from the PICC
  byte size = sizeof(buffer);
  Serial.println(F("Authenticating using key A..."));//Authenticate using key A
  status = (MFRC522::StatusCode) mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {Serial.print(F("PCD_Authenticate() failed: ")); Serial.println(mfrc522.GetStatusCodeName(status)); return; } // Show a Sector  Serial.println(F("Current data in sector 0:")); mfrc522.PICC_DumpMifareClassicSectorToSerial(&(mfrc522.uid), &key, sector); Serial.println(); // Read data from the block
    Serial.print(F("Reading data from block ")); Serial.print(blockAddr);Serial.println(F(" ..."));
    status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: ")); Serial.println(mfrc522.GetStatusCodeName(status));}
    dump_byte_array(buffer, 16); Serial.println();
    mfrc522.PICC_HaltA(); // Halt PICC
    mfrc522.PCD_StopCrypto1(); // Stop encryption on PCD
    String Data_from_Hex_to_ascii; //Print the Value once
    for (int i = 0; i < 16;i++) {  //Convert Hex Values to a String
      Data_from_Hex_to_ascii = Data_from_Hex_to_ascii + String(char(buffer[i]));  }      // Serial.println(Data_from_Hex_to_ascii);
    //Send RFID Tag Data  // } //    const char *c = Data_from_Hex_to_ascii.c_str();
    //-----------------------
    float temp = abs(-1000*scale.get_units());
    int x = temp;
    String y = String(round(x));
    y.trim();    
    String z = Data_from_Hex_to_ascii;
    z.trim();
    z = z + y;
    //-----------------------
    const char *c = z.c_str();
    Serial.println(c);
    item = c;
    bleRFIDidValueCharacteristic->setValue(item); 
    bleRFIDidValueCharacteristic->notify();
}
  if(Serial.available()){
      ctrl = Serial.readStringUntil('\n');
      ctrl.trim();
      getDirection(ctrl);
}
    DistanceMaintaining();
    delay(50);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void getLoadReading(){
  scale.set_scale(calibration_factor); 
  float temp = abs(-1000*scale.get_units());
  int x = temp;
  String y = String(round(x));
  const char *o = y.c_str();
  weight = o;
  bleTotalWeightCharacteristic->setValue(weight);
  bleTotalWeightCharacteristic->notify();
  Serial.print(o);
  //Adjust to this calibration factor //   if(Serial.available()){ // char temp = Serial.read(); // if(temp == 't') scale.tare(); //   }
  Serial.print(" Reading: "); Serial.print(-1*scale.get_units(),4); Serial.print(" kg");  Serial.println();
  }
//****************************************************************************************
void dump_byte_array(byte *buffer, byte bufferSize) {//buffer stores all the values
    for (byte i = 0; i < bufferSize; i++) {
        Serial.print(buffer[i] < 0x10 ? " 0" : " ");
        Serial.print(buffer[i], HEX);
    }
}
//****************************************************************************************
void getDirection(String ctrl){
  if(ctrl.equals("left")){
    myservo.write(50);
    subject = 1;}
  else if (ctrl.equals("left5")){
    myservo.write(65);
    subject = 1;}
  else if (ctrl.equals("center")){
    myservo.write(90);
    subject = 1;}
  else if (ctrl.equals("right5")){
    myservo.write(115);
    subject = 1;}
  else if (ctrl.equals("right")){
    myservo.write(130);
    subject = 1;}
  else if (ctrl.equals("ufo")){
    myservo.write(90);
    subject = 0;}  
} 
//*****************************************************************************************
void steer(int steerVal){
  switch (steerVal) {
  case 1:
    myservo.write(50);
    break;
  case 0:
    myservo.write(90);
    break;
  case 2:
    myservo.write(130);
    break;
}
}
//*****************************************************************************************
void SonarSensor(int trigPin,int echoPin){ 
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH);
distance = (duration/2) / 29.1;
}
//*****************************************************************************************
void MotorControl(char motor){
  digitalWrite(enable1Pin, HIGH);
    switch (motor) {
  case 'f':
    //Serial.println("Moving Forward");// Move the DC motor forward at maximum speed  
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);    
    break;
  case 'r':
    //Serial.println("Moving Reverse");// Move the DC motor forward at maximum speed  
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    break;
  case 'h':
    Serial.println("Motor Halted");// Stop the DC motor
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    break;
}
}       
//*****************************************************************************************
void DistanceMaintaining(){
  int lowerBound = 50;
  int upperBound = 80;
  SonarSensor(trigPin2, echoPin2);
  MiddleSensor = distance;
 if (distance > lowerBound && distance < upperBound){//In range
  MotorControl('h');
 }
 else if (distance >= upperBound ){  
  MotorControl('f');
  }
 else if (distance <= lowerBound && distance > 8){
    if      (subject == 1 &&  ctrl.equals("center")){
    MotorControl('h');  
    steer(0);  
    }
    else if (subject == 1 && !ctrl.equals("center")){
    MotorControl('h');
    steer(0); 
    }
    else if (subject == 0){//Halt Totally if no one present
    decideDir();
    MotorControl('f');   
    }
  }
 else if (distance <= 8){
  MotorControl('r');
  steer(0); 
  }
}
//*****************************************************************************************
bool decideDir(){
  SonarSensor(trigPin3, echoPin3);
  RightSensor = distance;
  SonarSensor(trigPin1, echoPin1);
  LeftSensor = distance;
  if (LeftSensor > RightSensor)
  {
    Serial.print("Left");
    Serial.println(LeftSensor);
    steer(2) ;
    return true;}
  else if (LeftSensor < RightSensor){
    Serial.print("Right");
    Serial.println(RightSensor); 
    steer(1);
    return false;}
  }
//*****************************************************************************************
void SensorData(){
  SonarSensor(trigPin1, echoPin1);
  RightSensor = distance;
  SonarSensor(trigPin2, echoPin2);
  MiddleSensor = distance;
  SonarSensor(trigPin3, echoPin3);
  LeftSensor = distance;
  Serial.print(LeftSensor);Serial.print(" - ");Serial.print(MiddleSensor);Serial.print(" - ");Serial.println(RightSensor); 
}
