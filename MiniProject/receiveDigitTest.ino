/*
Andrew Maginity
EENG - 350
Assignment 2 - Exercise 4

Purpose:  Sends a user input message to the arduino via i2c connection
          by converting it to an array of numbers, then receives the
          message reversed and converts it back to a string. Prints the
          sent and received messages to the LCD screen.
Harware:  Arduino and LCD screen connected to raspberry pi using
          i2c connection with common ground and VCC from the pi
          to the LCD.
Running:  Run the program and enter a message when prompted to send the
          message to the arduino and receive the reversed message, the
          results will be printed to the pi and the LCD screen.
*/

#include <Wire.h>

#define SLAVE_ADDRESS 0x04
byte numbers;
byte message[16];
byte revMessage[16];
int cnt = 0;

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600); // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
}

void loop() {
  delay(100);
}

// callback for received data
void receiveData(int byteCount){
  //loop to clear anthing stored in arrays
  for (int i = 0; i<16; i++){
    message[i] = 0;
    revMessage[i] = 0;
  }
  cnt = 0;  //reset cnt
  //loop to read from pi via i2c, stored as array 'message'
  while(Wire.available()) {
    numbers = Wire.read();    
    message[cnt++] = numbers;
  }
  //loop to create reversed array from what was received
  for (int i = 0; i<=cnt; i++){
    revMessage[i] = message[cnt - i];
  } 
  cnt = 0;       //reset cnt for writing message
}

// callback for sending data
void sendData(){
  if (cnt < 16){  
    Wire.write(revMessage[cnt++]);
  }
}
