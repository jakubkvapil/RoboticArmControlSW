#include <Servo.h> // Knihovna pro ovladani servomotoru 
#include <HX711_ADC.h> // Knihovna pro ovladani senzoru

// Definice pinu DIR a STEP krokovych motoru
const int stepPinA = 52; 
const int dirPinA = 53; 
const int stepPinB = 2; 
const int dirPinB = 5;
const int stepPinC = 3; 
const int dirPinC = 6; 
const int stepPinD = 4; 
const int dirPinD = 7;

// Definice pinu koncovych spinacu
const int endStopPinA = 30;
const int endStopPinB = 11;
const int endStopPinC = 10;
const int endStopPinD = 9;

// Limity jednotlivych os
const int limitStepsAMin = 0;
const int limitStepsBMin = 0;
const int limitStepsCMin = 0;
const int limitStepsDMin = 0;
const int limitStepsAMax = 2900;
const int limitStepsBMax = 780;
const int limitStepsCMax = 1230;
const int limitStepsDMax = 1540;

// Definice pinu zeleného start tlačítka
const int greenBttnPin = 51;

// Senzor
const int HX711_dt = 36;
const int HX711_sck = 37;
const float calibrationValue = -2452.33;
HX711_ADC sensor1(HX711_dt, HX711_sck);
unsigned long stabilizingtime = 2000;
boolean tareBool = true;

// Servomotory
Servo servoGripperTurn;
Servo servoGripperSqueeze;
const int servoGripperTurnPin = 46;
const int servoGripperSqueezePin = 44; 

// Hodnota, ktera je poslana Py programem pres seriovou komunikaci
int pyStepsA; 
int pyStepsB;
int pyStepsC;
int pyStepsD;
int pyGripperAngle;
int pyWristAngle;
int pyCommand;
int delaySteppers;
int delayServos;

// Aktualni hodnoty
int absStepsA;
int absStepsB;
int absStepsC;
int absStepsD;
int absGripperAngle = 180;
int absWristAngle = 60;

bool serialMarker;

unsigned long timeKeeperGrnBttn;
int pressedTimeGrnBttn;
int limitGrnBttn = 100;
bool GrnBttnWhile = true;

unsigned long timeKeeperEndStop;
int pressedTimeEndStop;
int limitEndStop = 30;
bool EndStopWhile = true;


// Serva
#include <Servo.h> 
Servo servoGripper;
Servo servoWrist;

void serialMarkerStart(){
  serialMarker = true;
}

void serialMarkerEnd(){
  if (serialMarker) {
  Serial.print("y\n");
  serialMarker = false;
}}

void(* resetArduino) (void) = 0; // Softwarová funkce pro reset Arduina

// Funkce pro kalibraci osy
void calibrateAxis(int dirPin, int stepPin, int endStopPin, bool dir) {
  digitalWrite(dirPin,dir);
  EndStopWhile = true;
  if (dirPin==dirPinA){
    while (EndStopWhile) {
     digitalWrite(stepPin,LOW);
     delayMicroseconds(4000);
     digitalWrite(stepPin,HIGH);
     delayMicroseconds(4000);
     timeKeeperEndStop = millis();
     while(!digitalRead(endStopPin)){
      pressedTimeEndStop = millis()-timeKeeperEndStop;
      if(pressedTimeEndStop >= limitEndStop){
        pressedTimeEndStop = 0;
        EndStopWhile = false;
        break;}}}
      }
   else {
     while (EndStopWhile) {
       digitalWrite(stepPin,LOW);
       delayMicroseconds(25000);
       digitalWrite(stepPin,HIGH);
       delayMicroseconds(25000);
       timeKeeperEndStop = millis();
       while(!digitalRead(endStopPin)){
        pressedTimeEndStop = millis()-timeKeeperEndStop;
        if(pressedTimeEndStop >= limitEndStop){
          pressedTimeEndStop = 0;
          EndStopWhile = false;
          break;}}}
          } 
          }

// Funkce pro cteni hodnoty senzoru
void readSensor1Value() {
  for (int i = 0; i <= 2000; i++) {
    sensor1.update();
    delay(1);}
    
  float mass = sensor1.getData();
  serialMarkerEnd();
  delay(20);
  Serial.println(mass);
  }

// Tare funkce senzoru
void tareSensor1() {
 sensor1.tareNoDelay();
 delay(20);
 serialMarkerEnd();
}

// Funkce pro kalibraci jednotlivych os
void calibrateA() {
  calibrateAxis(dirPinA,stepPinA,endStopPinA,LOW);
  absStepsA = 0;
  pyStepsA = 0;}

void calibrateB() {
  calibrateAxis(dirPinB,stepPinB,endStopPinB, LOW);
  absStepsB = 0;
  pyStepsB = 0;}

void calibrateC() {
  calibrateAxis(dirPinC,stepPinC,endStopPinC,LOW);
  absStepsC = 0;
  pyStepsC = 0;}

void calibrateD() {
  calibrateAxis(dirPinD,stepPinD,endStopPinD,HIGH);
  absStepsD = 0;
  pyStepsD = 0;}

// Funkce pro kalibraci vsech os  
void calibrateAll() {
 calibrateB();
 calibrateC();
 calibrateD();
 calibrateA();}

// Funkce pro ovladani servomotoru  
void moveServos(){
  int relDegreesGripperSqueeze = pyGripperAngle - absGripperAngle;
  int relDegreesGripperTurn = pyWristAngle - absWristAngle;

  if (abs(relDegreesGripperTurn)==relDegreesGripperTurn) { 
    for(int i = absWristAngle; i <= pyWristAngle; i += 1){
      servoGripperTurn.write(i);
      delay(delayServos);}  
  }  
  else {
      for(int i = absWristAngle; i >= pyWristAngle; i -= 1){
      servoGripperTurn.write(i);
      delay(delayServos);}   
  }

  if (abs(relDegreesGripperSqueeze)==relDegreesGripperSqueeze) { 
  
    for(int i = absGripperAngle; i <= pyGripperAngle; i += 1){
        servoGripperSqueeze.write(i);
        delay(delayServos);} 
  }
  else {
    for(int i = absGripperAngle; i >= pyGripperAngle; i -= 1){
        servoGripperSqueeze.write(i);
        delay(delayServos);} 
  }
  absGripperAngle = pyGripperAngle; 
  absWristAngle = pyWristAngle; 
  serialMarkerEnd();   
  }
 
// Funkce pro ovladani krokovych motoru
void moveSteppers() {
  if (((pyStepsA>=limitStepsAMin) && (pyStepsA<=limitStepsAMax)) && ((pyStepsB>=limitStepsBMin) && (pyStepsB<=limitStepsBMax)) && ((pyStepsC>=limitStepsCMin) && (pyStepsC<=limitStepsCMax)) && ((pyStepsD>=limitStepsDMin) && (pyStepsD<=limitStepsDMax))) {
  
  if (pyStepsA!=absStepsA || pyStepsB!=absStepsB || pyStepsC!=absStepsC || pyStepsD!=absStepsD) {
  int relStepsA;
  int relStepsB;
  int relStepsC;
  int relStepsD;
  
  relStepsA = pyStepsA - absStepsA;
  if (abs(relStepsA)==relStepsA) { digitalWrite(dirPinA,HIGH);}
  else { digitalWrite(dirPinA,LOW);}
  relStepsA = abs(relStepsA);

  relStepsB = pyStepsB - absStepsB;
  if (abs(relStepsB)==relStepsB) { digitalWrite(dirPinB,HIGH);}
  else { digitalWrite(dirPinB,LOW);}
  relStepsB = abs(relStepsB); 

  relStepsC = pyStepsC - absStepsC;
  if (abs(relStepsC)==relStepsC) { digitalWrite(dirPinC,HIGH);}
  else { digitalWrite(dirPinC,LOW);}
  relStepsC = abs(relStepsC); 

  relStepsD = pyStepsD - absStepsD;
  if (abs(relStepsD)==relStepsD) { digitalWrite(dirPinD,LOW);}
  else { digitalWrite(dirPinD,HIGH);}
  relStepsD = abs(relStepsD); 

  delayMicroseconds(1000);
  while ( (relStepsA > 0)||(relStepsB > 0)||(relStepsC > 0)||(relStepsD > 0) ) {
    if (relStepsA > 0) {
      digitalWrite(stepPinA,HIGH); 
    } 
    if (relStepsB > 0) {
      digitalWrite(stepPinB,HIGH); 
    }
    if (relStepsC > 0) {
      digitalWrite(stepPinC,HIGH); 
    } 
    if (relStepsD > 0) {
      digitalWrite(stepPinD,HIGH); 
    }
    delayMicroseconds(delaySteppers*1000); 
    if (relStepsA > 0) { 
      digitalWrite(stepPinA,LOW); 
      relStepsA--;
    } 
    if (relStepsB > 0) { 
      digitalWrite(stepPinB,LOW); 
      relStepsB--;
    }
      if (relStepsC > 0) { 
      digitalWrite(stepPinC,LOW); 
      relStepsC--;
    } 
    if (relStepsD > 0) { 
      digitalWrite(stepPinD,LOW); 
      relStepsD--;
    }
    delayMicroseconds(delaySteppers*1000); 
  }
absStepsA = pyStepsA;
absStepsB = pyStepsB;
absStepsC = pyStepsC;
absStepsD = pyStepsD;
serialMarkerEnd();}
 else {
serialMarkerEnd();
  }}
  else {
pyStepsA = absStepsA;
pyStepsB = absStepsA;
pyStepsC = absStepsC;
pyStepsD = absStepsD;

if (serialMarker) {
  Serial.print("n\n");
  Serial.print("Hodnoty natoceni motoru jsou mimo rozsah. Prikaz nebyl proveden.\n");
  serialMarker = false;}
}}

void setup() {
 Serial.begin(115200);
 pinMode(stepPinA,OUTPUT); 
 pinMode(dirPinA,OUTPUT);
 pinMode(stepPinB,OUTPUT); 
 pinMode(dirPinB,OUTPUT);
 pinMode(stepPinC,OUTPUT); 
 pinMode(dirPinC,OUTPUT);
 pinMode(stepPinD,OUTPUT); 
 pinMode(dirPinD,OUTPUT);
 
 pinMode(greenBttnPin,INPUT_PULLUP);
 pinMode(endStopPinD,INPUT_PULLUP);
 pinMode(endStopPinC,INPUT_PULLUP);
 pinMode(endStopPinB,INPUT_PULLUP);

 servoGripperSqueeze.attach(servoGripperSqueezePin);  
 servoGripperTurn.attach(servoGripperTurnPin);

 pyCommand = 0;
 
// Cekani, dokud neni stisknuto zelene start tlacitko
while (GrnBttnWhile) {
 timeKeeperGrnBttn = millis();
 while(!digitalRead(greenBttnPin))
 {
    pressedTimeGrnBttn = millis()-timeKeeperGrnBttn;
 
  if(pressedTimeGrnBttn >= limitGrnBttn)
  {
    pressedTimeGrnBttn = 0;
    GrnBttnWhile = false;
    break;
  }}}

 sensor1.begin();
 sensor1.start(stabilizingtime, tareBool);
 sensor1.setCalFactor(calibrationValue);
 
 servoGripperSqueeze.write( absGripperAngle);
 servoGripperTurn.write( absWristAngle);
 Serial.print("Arduino pripraveno\n");
}

void loop() {
    switch (pyCommand) {
    case 0:
        checkSerial();
        break;
    case 1:
        calibrateAll();
        serialMarkerEnd();
        pyCommand = 0;
        break;
    case 2:
        calibrateA();
        serialMarkerEnd();
        pyCommand = 0;
        break;
    case 3:
        calibrateB();
        serialMarkerEnd();
        pyCommand = 0;
        break;
    case 4:
        calibrateC();
        serialMarkerEnd();
        pyCommand = 0;
        break;
    case 5:
        calibrateD();
        serialMarkerEnd();
        pyCommand = 0;
        break;
    case 6:
        moveSteppers();
        pyCommand = 0;
        break;

    case 7:
        moveServos();
        pyCommand = 0;
        break;

    case 8:
        readSensor1Value();
        pyCommand = 0;
        break;

    case 9:
        tareSensor1();
        pyCommand = 0;
        break;

    case 199:
        serialMarkerEnd();
        delay(1000);
        resetArduino();
        pyCommand = 0;
        break;
        
    case 200:
        serialMarkerEnd();
        pyCommand = 0;
        break;
    }}

void checkSerial() {
if (Serial.available()==13) {
    serialMarkerStart();
    int stepsA1 = Serial.read();
    int stepsA2 = Serial.read();
    int stepsB1 = Serial.read();
    int stepsB2 = Serial.read();
    int stepsC1 = Serial.read();
    int stepsC2 = Serial.read();
    int stepsD1 = Serial.read();
    int stepsD2 = Serial.read();   
    pyGripperAngle = Serial.read();
    pyWristAngle = Serial.read();
    pyCommand = Serial.read();
    delaySteppers = Serial.read();
    delayServos = Serial.read();
    pyStepsA = stepsA1+stepsA2*256;
    pyStepsB = stepsB1+stepsB2*256;
    pyStepsC = stepsC1+stepsC2*256;
    pyStepsD = stepsD1+stepsD2*256;
}}
