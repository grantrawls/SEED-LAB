//SEED Group 10 
//Will Wikowsky
//Jaden Ranzenberger
//Andrew Maginity
//Grant Rawls
//Fall 2020
//This code allows the robot to search for a beacon, drive towards the beacon, and encompass the beacon.

#include "Arduino.h"
#include <Wire.h>

//Red - motor power (connects 1 motor terminal) 
//black - motor power (connects other motor terminal) 
//green - encoder Ground (Connect to GND)
//Blue - encoder VCC (Connect to VCC)
//Yellow - encoder A output
//White Encoder B output 

//Motor direction inputs
int m1dir = 7; //Left Wheel
int m2dir = 8; //Right Wheel
//Low will disable both motors, toggling will reset latched driver fault condition
int disable = 4;
//PWM signals for motors
const int m1pwm = 9;
const int m2pwm = 10;
//Status indicator of motors
const int sf = 13;
//Motor 1 current sense output (approx. 525 mV/A)
const int m1fb = 0;
//Motor 2 current sense output (approx. 525 mV/A)
const int m2fb = 1;

//Encoder 1 Pin Config 
const int outputA1 = 2;
const int outputA2 = 11;

//Encoder 2 Pin Config 
const int outputB1 = 3;
const int outputB2 = 12;

//Variables for encoderA and encoderB ISRs and associated calculations
int countsA = 0;
int countsB = 0;
int tempCountsA = 0;
int tempCountsB = 0;
double positionA = 0;
double positionB = 0;
double prevPositionA = 0;
double prevPositionB = 0;
double velocityA = 0;
double velocityB = 0;
int ISRtimeA = 0;
int ISRtimeB = 0;
int prevISRtimeA = 0;
int prevISRtimeB = 0;

//Non-variable control signal variables
int maxCtrlRho = 40; //max PWM signal
int maxCtrlPhi = 30; 
int minCtrl = 25; //minimum PWM signal to move robot consistently forward
int motorSpeed = 40; //PWM signal out of 255
int easySpeed = 60;
const int period = 20;  

//Variables for Forward Controller
double choosePosition = 0.3048; //meters
int currPosTime = 0; 
int prevPosTime = 0;
int deltaPosTime = 0;
double rhoOutput = 0;
double setPosition = 0;
double currPosition = 0;
double posError = 0;
double prevPosError = 0;
double totalPosError = 0;
double ratePosError = 0;
double deltaPosError = 0;
double prevPosTotalError = 0;

//Variables for Angular Controller
//double chooseAngle = PI/2; //radians
//double chooseAngle = 3*PI/4;
double chooseAngle = PI;
int currAngleTime = 0; 
int prevAngleTime = 0;
int deltaAngleTime = 0;
double setAngle = 0;
double currAngle = 0;
double angleError = 0;
double prevAngleError = 0;
double totalAngleError = 0;
double rateAngleError = 0;
double deltaAngleError = 0;
double prevAngleTotalError = 0;

double rhoKp = 10; // V/rad
double rhoKi = 0.5; // V/(rad*sec)
double posOutput = 0; 
double rWheel = 0.06985; //meters
double dWheels = 0.269875; //meters

double phiKp = 10; // V/rad
double phiKi = 0.5; // V/(rad*sec)
double phiInput = 0;
double angleOutput = 0; 

//Encompass function specific variab;es
double phiDot = 0;
double rhoDot = 0;
double rhoSetPosition = 0;
double rhoInput = 0;
int resetCountsA = 0;
int resetCountsB = 0;

//Global Flags
bool goForward = 0;
bool notGoneForward = 1;
bool runFirst = 1;
bool runSecond = 1;

bool corrected = 0;
bool doneSearching = 0;
bool oneFootAway = 0;
bool hasEncompassed = 0;

#define SLAVE_ADDRESS 0x04
byte message; //read from Pi

void setup() {
  // put your setup code here, to run once:
  pinMode(outputA1, INPUT_PULLUP);
  pinMode(outputB1, INPUT_PULLUP);
  pinMode(disable, OUTPUT);
  pinMode(m1dir, OUTPUT);
  pinMode(m2dir, OUTPUT);
  pinMode(m1pwm, OUTPUT);
  pinMode(m2pwm, OUTPUT);
  pinMode(sf, INPUT);
  digitalWrite(disable, HIGH);
  Serial.begin(115200);
  //position and velocity reading scheme and interrupt declaration
  attachInterrupt(digitalPinToInterrupt(outputA1), encoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB1), encoderBISR, CHANGE);
  //Information I/O to and from Pi
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  //Wire.onRequest(sendData);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (doneSearching && (!hasEncompassed)) //Beacon is found (Pi sends message = 1)
  {
    if(!corrected)
    {
    Serial.println("CORRECTING");
    rotateToPhi(-PI/16);
    corrected = 1;
    }
    Serial.println("GOING FORWARDS");
    easyGoForwards(); 
    if(oneFootAway) //when camera tells the robot it is one foot away (Pi sends message = 2)
    {
      //digitalWrite(disable, LOW); //For demo 2.1
      Serial.println("ONE FOOT");
      if (!hasEncompassed)
      {
         Serial.println("ENCOMPASSING");
         encompass(); //go in a diamond around the beacon
         hasEncompassed = 1;
      }
    }
  } else if(!hasEncompassed) //only search at the start
    {
      //rotating to find beacon
      Serial.println("SEARCHING");
      phiCalcPI(30*PI); //spin for a long time at a slow rate
    }
}

void encoderAISR(void) //LEFT WHEEL
{
  ISRtimeA = millis();
  //Compare A to B and count accordingly. 
  if (digitalRead(outputA1) == digitalRead(outputA2))
  {
    countsA -= 2; //CCW
  }
  else
  {
    countsA += 2; //CW
  }
  //Motor A calculations 
  //Calculate Position in rad
  positionA = (countsA*2*PI)/3020; // change 3200 to 64 if encoder counting in encoder counts not motor counts
  
  //Calculate velocity in rad/s
  velocityA = (positionA - prevPositionA)/(ISRtimeA - prevISRtimeA); //in rad/ms
  velocityA = velocityA*1000; //in rad/s
  
  prevPositionA = positionA; //set previous position
  prevISRtimeA = ISRtimeA;
}

void encoderBISR(void) //RIGHT WHEEL
{
  ISRtimeB = millis();
  //Compare A to B and count accordingly.
  //NOTE: Counts opposite of left motor because of orientation to keep counts positive when going forward.
  if (digitalRead(outputB1) == digitalRead(outputB2))
  {
    countsB += 2; //CCW
  }
  else
  {
    countsB -= 2; //CW
  }
  //Motor B calculations 
  //Calculate Position in rad
  positionB = (countsB*2*PI)/3020; // change 3200 to 64 if encoder counting in encoder counts not motor counts
  
  //Calculate velocity in rad/s
  velocityB = (positionB - prevPositionB)/(ISRtimeB - prevISRtimeB); //in rad/ms
  velocityB = velocityB*1000; //in rad/s
  prevPositionB = positionB; //set previous position
  prevISRtimeB = ISRtimeB;
}

void rhoCalcPI(double setPosition)
{

  Serial.println("ENTERED RHO PID");
  //Set robot to go forward

  //checking if one foot away
  //if(oneFootAway)
    //{
     // rhoInput = setPosition;
    //}
  if(posError > 0)
  {
  digitalWrite(m1dir, LOW);
  digitalWrite(m2dir, HIGH);
  }else
  {
  digitalWrite(m1dir, HIGH);
  digitalWrite(m2dir, LOW);
  }
  currPosTime = millis();
  deltaPosTime = currPosTime - prevPosTime;
  rhoInput = (positionA + positionB)/2;
  rhoInput = rhoInput*rWheel;
  
  if(countsA > 14000 && countsB > 14000)
  {
    setPosition = setPosition - rhoInput;
    countsA = 0;
    countsB = 0;
  }

  if(deltaPosTime >= period)
  {
    posError = setPosition - rhoInput;///
    totalPosError += posError; //Integrand
    deltaPosError = posError - prevPosError;
    posOutput = rhoKp*posError + rhoKi*deltaPosError*period; //V bar (Volts)
    posOutput = (256*(posOutput/24)) - 1; //PWM signal change for 12 volt motor (battery at 8 though?) and split in half because two wheels.

    if (posOutput > maxCtrlRho)
    {
      posOutput = maxCtrlRho;
    }else if(posOutput < minCtrl)
    {
      posOutput = minCtrl;
    }if(posError > -0.01 && posError < 0.01)
    {
      posOutput = rhoKp*posError + rhoKi*deltaPosError*period; //V bar (Volts)
      runSecond = 0;
      goForward = 0;
      oneFootAway = 1;
      //countsA = 0;
      //countsB = 0;
    }
    prevPosError = posError;
    prevPosTime = currPosTime;
    prevPosTotalError = totalPosError;
  }
//    Serial.print(rhoInput);
//    Serial.print("\t");
//    Serial.print(posError);
//    Serial.print("\t");
//    Serial.print(totalPosError);
//    Serial.print("\t");
//    Serial.print(posOutput);
//    Serial.print("\t");
//    Serial.print(countsA);
//    Serial.print("\t");
//    Serial.print(countsB);
//    Serial.println("");
  analogWrite(m1pwm, posOutput); //motor A input
  analogWrite(m2pwm, posOutput); //motor B input
}


void phiCalcPI(double setAngle)
{
  //Set robot to turn LEFT
  if(angleError > 0.01)
  {
  digitalWrite(m1dir, HIGH);
  digitalWrite(m2dir, HIGH);
  }else if(angleError < -0.01) //set to turn RIGHT
  {
  digitalWrite(m1dir, LOW);
  digitalWrite(m2dir, LOW);
  }
  currAngleTime = millis();
  deltaAngleTime = currAngleTime - prevAngleTime;
  phiInput = (-1*((positionA - positionB)/2))*(2*rWheel/dWheels);
  //phiInput = (2*phiInput*rWheel)/dWheels;
  if(countsA > 14000 && countsB > 14000)
    {
      setAngle = setAngle - phiInput;
      countsA = 0;
      countsB = 0;
    }
  //fix this for phi controller (if statements)
  if(deltaAngleTime >= period)
  {
    angleError = setAngle - phiInput;
    totalAngleError += angleError; 
    deltaAngleError = angleError - prevAngleError;
    //rateError = (posError - prevError)/deltaTime;
    angleOutput = phiKp*angleError + phiKi*deltaAngleError*period; //V bar (Volts)
    angleOutput = (256*(angleOutput/24)) - 1; //PWM signal change for 12 volt motor (battery at 8 though?) and split in half because two wheels.

    if (angleOutput > maxCtrlPhi)
    {
      angleOutput = maxCtrlPhi;
    }else if(angleOutput < minCtrl)
    {
      angleOutput = minCtrl;
    }if(angleError > -0.01 && angleError < 0.01)
    {
      angleOutput = phiKp*angleError + phiKi*deltaAngleError*period; //V bar (Volts
      doneSearching = 1;
      countsA = 0;
      countsB = 0;
    }
    prevAngleError = angleError;
    prevAngleTime = currAngleTime;
    prevAngleTotalError = totalAngleError;
  }
//    Serial.print(phiInput);
//    Serial.print("\t");
//    Serial.print(angleError);
//    Serial.print("\t");
//    Serial.print(totalAngleError);
//    Serial.print("\t");
//    Serial.print(angleOutput);
//    Serial.print("\t");
//    Serial.print(countsA);
//    Serial.print("\t");
//    Serial.print(countsB);
//    Serial.println("");
  analogWrite(m1pwm, angleOutput); //motor A input
  analogWrite(m2pwm, angleOutput); //motor B input
  
}

void rotateToPhi(double phiSetPosition){
  Serial.println("ROTATION FUNCTION STARTED");
    // rhoInput is the current forward position
  countsA = 0;
  countsB = 0;
  phiDot = 0;
  rhoInput = (positionA + positionB)/2;
  rhoInput = rhoInput*rWheel;
  //Current angular position (radians)
//  
  Serial.print(phiSetPosition);
  Serial.print("\t");
  Serial.print(phiDot);
  Serial.println("");
  while(!(phiSetPosition - phiDot <= .05 && phiSetPosition - phiDot >=-.05)){
//    Serial.print(phiSetPosition);
//    Serial.print("\t");
//    Serial.print(phiDot);
//    Serial.println("");
    //Serial.print(countsA);
    //Serial.print("\t");
    //Serial.print(countsB);
    //Serial.println("");
    phiDot = (rWheel*(positionA - positionB))/dWheels;
      if(countsA > 14000 && countsB > 14000)
    {
      phiSetPosition = phiSetPosition - phiDot;
      countsA = 0;
      countsB = 0;
    }
  if(phiSetPosition - phiDot <= .05 && phiSetPosition - phiDot >=-.05){
    digitalWrite(m1dir, LOW);
    digitalWrite(m2dir, HIGH);
    analogWrite(m1pwm, 0);
    analogWrite(m2pwm, 0);
    resetCountsA = countsA;
    resetCountsB = countsB;
    countsA = resetCountsA;
    countsB = resetCountsB;
    } else if(phiSetPosition - phiDot >.01){
    digitalWrite(m1dir, LOW);
    digitalWrite(m2dir, LOW);
    analogWrite(m1pwm, motorSpeed);
    analogWrite(m2pwm, motorSpeed);
    } else {
    digitalWrite(m1dir, HIGH);
    digitalWrite(m2dir, HIGH);
    analogWrite(m1pwm, motorSpeed);
    analogWrite(m2pwm, motorSpeed); 
    }
  }
     digitalWrite(m1dir, LOW);
    digitalWrite(m2dir, HIGH);
    analogWrite(m1pwm, 0);
    analogWrite(m2pwm, 0);
  }
double goForwards(double distance){
  //rhoInput is currentPosition
  countsA = 0;
  countsB = 0;
  rhoInput = 0;
  //rhoSetPosition is the goal position
  rhoSetPosition = rhoInput + distance;
  Serial.println("FORWARDS FUNCTION STARTED");
  while(rhoSetPosition - rhoInput >= .02 || rhoSetPosition - rhoInput <=-.02){
    Serial.print(rhoSetPosition);
    Serial.print("\t");
    Serial.print(rhoInput);
      Serial.println("");
    if(countsA > 14000 && countsB > 14000)
    {
      rhoSetPosition = rhoSetPosition - rhoInput;
      countsA = 0;
      countsB = 0;
    }
    //Serial.print(countsA);
    //Serial.print("\t");
    //Serial.print(countsB);
    //Serial.println("");
  if(rhoSetPosition - rhoInput <= .02 && rhoSetPosition - rhoInput >=-.02){
    //Serial.print("\t");
    //Serial.println("Made it.");
    digitalWrite(m1dir, LOW);
    digitalWrite(m2dir, LOW);
    analogWrite(m1pwm, 0);
    analogWrite(m2pwm, 0);  
    } else if(rhoSetPosition - rhoInput >.01){
//     Serial.print("\t");
//     Serial.print("Not there yet.");
    digitalWrite(m1dir, LOW);
    digitalWrite(m2dir, HIGH);
    analogWrite(m1pwm, motorSpeed);
    analogWrite(m2pwm, motorSpeed);
    } else {
//    Serial.print("\t");
//    Serial.println("Overshoot.");
    digitalWrite(m1dir, HIGH);
    digitalWrite(m2dir, LOW);
    analogWrite(m1pwm, motorSpeed);
    analogWrite(m2pwm, motorSpeed); 
    }
   // Serial.print(rhoInput);
    //Serial.print("\t");
    //Serial.print(rhoSetPosition);
    //Serial.println("");  

    rhoInput = (positionA + positionB)/2;
    rhoInput = rhoInput*rWheel;
  }
      digitalWrite(m1dir, LOW);
    digitalWrite(m2dir, HIGH);
    analogWrite(m1pwm, 0);
    analogWrite(m2pwm, 0);

 }

void encompass(void)
{
  motorSpeed = 80;
  //positive angle goes right just for this function
  rotateToPhi(PI/5.8);
  goForwards(.3096*1.1);
  rotateToPhi(-PI/5);
  goForwards(.3096*1.1);
  rotateToPhi(-PI/4.5);
  goForwards(.3096*1);
  rotateToPhi(-PI/4.5);//4.2
  goForwards(.3096*0.5);
}
void receiveData(int byteCount){
  //loop to read from pi via i2c, stored as 'message'
    message = Wire.read(); 
 
    switch (message){ //Need to make it so motor can spin the other way to go back to zero or normalize position within 2pi rad because the position can just keep increasing in one direction
 
      case 1:
        doneSearching = 1;
        break;
        
      case 2:
        oneFootAway = 1;
        //Serial.print("Flag set: ");
        //Serial.println(oneFootAway);
        break;

    }
//    if(!doneSearching)
//    {
//      Serial.println(message);
//    }else if(doneSearching && oneFootAway)
//      {
//      Serial.println(message);
//      }
}

// callback for sending data
//void sendData(){
   // Wire.write(choosePos);
//}

void easyGoForwards(void)
{
  digitalWrite(m1dir, LOW);
  digitalWrite(m2dir, HIGH);
  analogWrite(m1pwm, easySpeed);
  analogWrite(m2pwm, easySpeed);
}
