int period = 10; //10 ms sample peiod
const int commandVoltage = 1; //in Volts
int voltage = ; //voltage input on pin A0
int voltVal = 0;
int prevPos = 0;    // in radians
int angPos = 0; // in radians
int motorPos = 0;
int angVel = 0; // in rad/sec
unsigned long currTime = 0;
const int encA = 2; //Connected to pin 2
const int encB = 3; //Connected to pin 3
int valA;           //Track A and B
int valB;
int counts = 0;     //Track counts

void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200);
   Encoder motorEncoder(encA, encB
   pinMode(voltage, OUTPUT);
   //pinMode(encA, INPUT_PULLUP); 
   //pinMode(encB, INPUT_PULLUP);
   //attachInterrupt(digitalPinToInterrupt(encA), encoderISR, CHANGE); //Set encoderISR interrupt to trigger on a change in A
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if(millis() >= 1000 & millis() < 2000)
  { 
    analogWrite(voltage, commandVoltage);
    prevPos = angPos;
    currTime = millis();
    Serial.print("Current Time (ms): ");
    Serial.println(currTime);
    Serial.print("Motor Voltage Command: ");
    Serial.println(analogRead(voltage));
    Serial.print("Angular Velocity: ");
    Serial.println(angVel);
    while(millis() < currTime + period)
    {
      motorPos = motorEncoder.read();
    }
    //angPos = 50*counts*2*3.14/3200;
    angPos = 50*motorPos*2*3.14/3200;
    angVel = (angPos-prevPos)/period;
    if(period < millis() - currTime)
    {
      Serial.println("ERROR: Sampling period too long!");
    }
    Serial.println("----------------------------------------");
  }
}
/*void encoderISR()
{
  valA = digitalRead(encA);
  valB = digitalRead(encB);
  
  //Compare A to B and count accordingly.
  if (valA == valB)
  {
    counts -= 2; //CCW
  }
  else
  {
    counts += 2; //CW
  }
  Serial.println("Total counts: ");
  Serial.println(counts);
}
//END of EncoderISR Sketch*/
