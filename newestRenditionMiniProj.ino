#include "Arduino.h"
#include <Encoder.h>
#include <Wire.h>

//Red - motor power (connects 1 motor terminal) command voltage? pin 9
//black - motor power (connects other motor terminal) ground?
//green - encoder Ground(Connect to GND)
//Blue - encoder VCC(Connect to VCC)
//Yellow - encoder A output (Connect to pin 2)
//White Encoder B output (Connect to pin 11)

float Kp = 2.149; // Unit: Volts/rad
float Kd = 17.817; // Unit: Volts/(rad/sec)
float Ki = 0.0391; // Unit: Volts/(rad*sec)
float I = 0;
float D = 0;
float error;
float prevError = 0;
const int period = 30; //30 ms sample period
float controlOutput = 0; // controller output signal
int newPos = 0;

// Tracking Variables
int pos0Counts = 0;
int pos1Counts = 0;
int pos2Counts = 0;
int pos3Counts = 0;
int rev = 1;
int check = 0;
unsigned long currTime = 0;

// I/O Pins
const int pin4 = 4;
const int encA = 2; //Connected to pin 2
const int encB =11; //Connected to pin 3
int m2dir = 8; //Using Motor 2
const int voltage = 10; //voltage output on pin 10

// Positions
int long motorPos = 0;
int desiredPos = 0; //desired position in encoder counts
int actualPos = 0; //actual position in encoder counts

Encoder motorEncoder(encA, encB); //declare the encoder pins using the Encoder.h library

#define SLAVE_ADDRESS 0x04
byte message; //message read from Pi
byte prevMsg;
int choosePos = 6; //Message sent back to Pi

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(voltage, OUTPUT);
  pinMode(m2dir, HIGH); //Positive direction
  pinMode(pin4, OUTPUT);
  digitalWrite(pin4, HIGH);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
}
//(actualPos + 3180) >= 0) && rev > 1)
//(((actualPos*-1) % 3200) == 0) && rev > 1)
void loop() {
  // put your main code here, to run repeatedly:
  while((abs(desiredPos - actualPos) >= 10))//abs(desiredPos - actualPos) >= 10)
  {
    actualPos = motorEncoder.read();
    if (actualPos < desiredPos)
    {
     
      analogWrite(voltage,0);
      actualPos = desiredPos - 12;
      //desiredPos = actualPos;
      break;
    }
//    if(((actualPos + 2100) >= 0) && rev > 1)
//  {
//    
//    rev += 1;
//    //actualPos = actualPos - ((rev-1)*3200);
//    //desiredPos = desiredPos - (3200*(rev-1));
//    //break;
//  }
 
    newPos = actualPos-desiredPos;
    newPos = Kp*newPos;
    
    newPos = newPos - newPos/currTime;
  
    
    analogWrite(voltage, newPos);
    if (millis() > currTime + period)
    {
      //Serial.print(currTime);
      //Serial.print("\t ");
      // Serial.print("Desired: ");
      Serial.println(desiredPos);
      //Serial.print("Actual: ");
      Serial.println(actualPos);
  
      currTime += period;
    }
  }
  analogWrite(voltage,0);
  /*delay(500);
  //check++;
  check += 1;
switch (check%4){ //Need to make it so motor can spin the other way to go back to zero or normalize position within 2pi rad because the position can just keep increasing in one direction
      case 0:
        desiredPos = -3000 - (3200*pos0Counts); // 0 rad in motor counts
        choosePos = 0;
        pos0Counts = pos0Counts + 1;
        break;
      case 1:
        desiredPos = -630 - (3200*pos1Counts); // pi/2 radin motor counts
        choosePos = 1;
        pos1Counts = pos1Counts + 1;
        break;
      case 2:
        desiredPos = -1400 - (3200*pos2Counts); // pi rad in motor counts
        choosePos = 2;
        pos2Counts = pos2Counts + 1;
        break;
      case 3:
        desiredPos = -2200 - (3200*pos3Counts); // 3pi/2 rad in motor counts
        choosePos = 3;
        pos3Counts = pos3Counts + 1;
        break;
    }*/
    
//  switch(check)
//  {
//  case 1:
//    desiredPos = -1400;
//    break;
//  case 2: 
//    desiredPos = -2200;
//    break;
//  case 3:
//    desiredPos = -3000;
//    break;
//  case 4:
//    desiredPos = -3830;
//    break;
//  case 5:
//    desiredPos = -4600;
//    break;
//  case 6:
//    desiredPos = -5400;
//    break;
//  }
}

void receiveData(int byteCount){
  //loop to read from pi via i2c, stored as 'message'
    message = Wire.read(); 

    if(message != prevMsg)
    {
      switch (message){ //Need to make it so motor can spin the other way to go back to zero or normalize position within 2pi rad because the position can just keep increasing in one direction
      case 0:
        desiredPos = 0 - (3200*(pos0Counts)); // 0 rad 
        choosePos = 0;
        pos0Counts = pos0Counts + 1;  //may need to use rev for all instead of posXCounts
        pos1Counts = pos1Counts + 1;
        pos2Counts = pos2Counts + 1;
        pos3Counts = pos3Counts + 1;
        break;
      case 1:
        desiredPos = -630 - (3200*(pos1Counts)); // pi/2 rad
        choosePos = 1;
        pos0Counts = pos0Counts + 1;  //may need to use rev for all instead of posXCounts
        pos1Counts = pos1Counts + 1;
        pos2Counts = pos2Counts + 1;
        pos3Counts = pos3Counts + 1;
        break;
      case 2:
        desiredPos = -1400 - (3200*(pos2Counts)); // pi rad 
        choosePos = 2;
        pos0Counts = pos0Counts + 1;  //may need to use rev for all instead of posXCounts
        pos1Counts = pos1Counts + 1;
        pos2Counts = pos2Counts + 1;
        pos3Counts = pos3Counts + 1;
        break;
      case 3:
        desiredPos = -2200 - (3200*(pos3Counts)); // 3pi/2 rad 
        choosePos = 3;
        pos0Counts = pos0Counts + 1;  //may need to use rev for all instead of posXCounts
        pos1Counts = pos1Counts + 1;
        pos2Counts = pos2Counts + 1;
        pos3Counts = pos3Counts + 1;
        break;
     
        
       }
    }
    
    prevMsg = message;
}

// callback for sending data
void sendData(){
    Wire.write(choosePos);
}
