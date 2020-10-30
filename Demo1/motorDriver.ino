#include <Encoder.h>

//Motor 1 direction input
int m1dir = 7;
//Motor 2 direction input
int m2dir = 8;
//Low will disable both motors, toggling will reset latched driver fault condition
int disable = 4;
//Speed signal for motor 1
int m1pwm = 9;
//Speed signal for motor 2
int m2pwm = 10;
//Status indicator of motors
int sf = 12;
//Motor 1 current sense output (approx. 525 mV/A)
int m1fb = 0;
//Motor 2 current sense output (approx. 525 mV/A)
int m2fb = 1;

//Encoder Pin  1
int outputA1 = 2;
int outputA2 = 11;

//Encoder Pin 2
int outputB1 = 4;
int outputB2 = 3;

//Current Encoder Positions
int currentRead1;
int currentRead2;

//Encoder Object Setup
Encoder myEnc1(outputA1,outputA2);
Encoder myEnc2(outputB1,outputB2);
long oldPosition1  = -999;
long oldPosition2  = -999;
  
void setup() {
  pinMode(4,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(12,INPUT);
  digitalWrite(4,HIGH);
  digitalWrite(7,HIGH);
  digitalWrite(8,HIGH);
  Serial.begin(9600);
}

void loop() {
  //Because m1dir and m2dir are specified to be in same direction (in setup loop), 
  //the robot will go forwards at half speed indefinitely.
  analogWrite(9,125);
  analogWrite(10,125);

  //Testing encoder reading values with the custom encoder library
    long newPosition1 = myEnc1.read();
    long newPosition2 = myEnc2.read();
  if (newPosition1 != oldPosition1) {
    oldPosition1 = newPosition1;
    Serial.println(newPosition1);
  }
  if (newPosition2 != oldPosition2) {
    oldPosition2 = newPosition2;
    Serial.println(newPosition2);
  }

  
}
