#include "Arduino.h"
//Robot Name: Timmy Turbo
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
//Speed signals for motors
const int m1pwm = 9;
const int m2pwm = 10;
//Status indicator of motors
const int sf = 13;
//Motor 1 current sense output (approx. 525 mV/A)
const int m1fb = 0;
//Motor 2 current sense output (approx. 525 mV/A)
const int m2fb = 1;

//Encoder 1 Pin Config (LEFT WHEEL)
const int outputA1 = 2;
const int outputA2 = 11;

//Encoder 2 Pin Config (RIGHT WHEEL)
const int outputB1 = 3;
const int outputB2 = 12;

#define ARRAY_SIZE 4
double arrayA[ARRAY_SIZE];
double arrayB[ARRAY_SIZE];
int indexA = 0;
int indexB = 0;
int counterA = 0;
int counterB = 0;
//Control outputs

int ctrlOut1 = 32; //1 Volts 
int ctrlOut2 = 32; 

double rWheel = 0.06985; //meters
double dWheels =  0.269875; //meters
//Encoder Object Setup
//Encoder myEnc1(outputA1,outputA2);
//Encoder myEnc2(outputB1,outputB2);
//long oldPosition1  = -999;
//long oldPosition2  = -999;

//Variables for encoder reading and associated calculations
int countsA = 0;
int countsB = 0;
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
double phiDot = 0;

//sampling
const int period = 20; 
int currTime = 0; 
int calcTime = 0; 

// varaibles for serial communication
String InputString = ""; // a string to hold incoming data
bool StringComplete = false;
bool docounting = false;

void setup() {
  pinMode(outputA1, INPUT_PULLUP);
  pinMode(outputB1, INPUT_PULLUP);
  pinMode(disable, OUTPUT);
  pinMode(m1dir, OUTPUT);
  pinMode(m2dir, OUTPUT);
  pinMode(m1pwm, OUTPUT);
  pinMode(m2pwm, OUTPUT);
  pinMode(sf, INPUT);
  digitalWrite(disable, HIGH);
  digitalWrite(m1dir, HIGH);
  digitalWrite(m2dir, HIGH);
  Serial.begin(115200);

  //interrupt declaration for position and velocity reading scheme
  attachInterrupt(digitalPinToInterrupt(outputA1), encoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB1), encoderBISR, CHANGE);
  // reserve 200 bytes for the inputString
  InputString.reserve(200);
}

void loop() { //include ISR to read encoder and calculate velocity
  // put your main code here, to run repeatedly:
  
  currTime = millis();
  analogWrite(m1pwm, ctrlOut1);
  analogWrite(m2pwm, ctrlOut2);
  phiDot = (rWheel*(velocityA - velocityB))/dWheels;
  calcTime = millis();
  if((indexA == 0 || indexA == 3))
  {
  Serial.print(currTime);
  Serial.print("\t ");
  Serial.print(phiDot);
  Serial.println("");
  }
  if (currTime > 2000)
  {
    Serial.print("Finished ");
  }
  delay(period - (calcTime - currTime));
}

void encoderAISR(void) //LEFT WHEEL
{
  //ISRtimeA = millis();
  indexA = counterA % 4;
  arrayA[indexA] = millis();
  //Compare A to B and count accordingly.
  if (digitalRead(outputA1) == digitalRead(outputA2))
  {
    countsA -= 2; //CCW
  }
  else
  {
    countsA += 2; //CW
  }
  
  //Motor A Calculations
  if(indexA == 0 || indexA == 3)
  {
    //Calculate Position in rad
    positionA = (countsA*2*PI)/3200; 
    //Calculate velocity in rad/s
    velocityA = (positionA - prevPositionA)/(arrayA[3] - arrayA[0]); //in rad/ms
    velocityA = velocityA*1000; //in rad/s
    prevPositionA = positionA;
  }
  counterA++;
  //prevISRtimeA = ISRtimeA;
}

void encoderBISR(void) //RIGHT WHEEL
{
  //ISRtimeB = millis();
  indexB = counterB % 4;
  arrayB[indexB] = millis();
  //Compare A to B and count accordingly.
  if (digitalRead(outputB1) == digitalRead(outputB2))
  {
    countsB += 2; //CCW
  }
  else
  {
    countsB -= 2; //CW
  }
  
  //Motor B Calculations
  if(indexB == 0 || indexB == 3)
  {
    //Calculate Position in rad
    positionB = (countsB*2*PI)/3200; 
    //Calculate velocity in rad/s
    velocityB = (positionB - prevPositionB)/(arrayB[3] - arrayB[0]); //in rad/ms
    velocityB = velocityB*1000; //in rad/s
    prevPositionB = positionB;
  }
  counterB++;
  //prevISRtimeB = ISRtimeB;
}
