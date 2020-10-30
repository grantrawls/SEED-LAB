#include <Encoder.h>

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

//Encoder 1 Pin Config 
const int outputA1 = 2;
const int outputA2 = 11;

//Encoder 2 Pin Config 
const int outputB1 = 3;
const int outputB2 = 12;


//int currentRead1;
//int currentRead2;
//int deltaTimeISRB = 0;
//Control outputs

int ctrlOut1 = 32; //0.5 Volts (1/24 of max PWM output)
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
double rhoDot = 0;

//Timing variables
const int period = 10;  
int currTime = 0; 
int calcTime = 0; 
int prevTime = 0;

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
  digitalWrite(m1dir, LOW);
  digitalWrite(m2dir, HIGH);
  Serial.begin(115200);

  //position and velocity reading scheme and interrupt declaration
  attachInterrupt(digitalPinToInterrupt(outputA1), encoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB1), encoderBISR, CHANGE);
  // reserve 200 bytes for the inputString
  InputString.reserve(200);
  //Serial.println("Ready!"); // Let anyone on the other end of the serial line know that Arduino is ready
}

void loop() { 
  // put your main code here, to run repeatedly:
  analogWrite(m1pwm, ctrlOut1);
  analogWrite(m2pwm, ctrlOut2);
  //Calculate Position in rad
  currTime = millis();
  if(currTime - prevTime > period)
  {
    prevTime = currTime;
    positionA = (countsA*2*PI)/3200; // change 3200 to 64 if encoder counting in encoder counts not motor counts
    positionB = (countsB*2*PI)/3200; // change 3200 to 64 if encoder counting in encoder counts not motor counts
    //Calculate velocity in rad/s
    velocityA = (positionA - prevPositionA)/(period); //in rad/ms
    velocityB = (positionB - prevPositionB)/(period); //in rad/ms
    velocityA = velocityA*1000; //in rad/s
    velocityB = velocityB*1000; //in rad/s
    prevPositionA = positionA; //set previous position
    prevPositionB = positionB; //set previous position
    //Calculate rhoDot, the forward velocity
    rhoDot = (rWheel*(velocityA + velocityB))/2;
    Serial.print(prevTime);
    Serial.print("\t ");
    Serial.print(rhoDot);
    Serial.println("");
//        Serial.println("");
//    Serial.println(countsA);
//    Serial.println(countsB);
  }

  
  if (currTime > 2000)
  {
    Serial.print("Finished ");
  }

  //delay(50 - (calcTime - currTime));
}

void encoderAISR(void) //LEFT WHEEL
{
  //ISRtimeA = millis();
  //Compare A to B and count accordingly. 
  if (digitalRead(outputA1) == digitalRead(outputA2))
  {
    countsA -= 2; //CCW
  }
  else
  {
    countsA += 2; //CW
  }
  //prevISRtimeA = ISRtimeA;
}

void encoderBISR(void) //RIGHT WHEEL
{
  //ISRtimeB = millis();
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
  //prevISRtimeB = ISRtimeB;
}
