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
double rhoDot = 0;
double rhoInput = 0;
double rhoOutput = 0;

const int period = 10;  
int currTime = 0; 
int prevTime = 0;
int elapsedTime = 0;
double setPosition = 0;
double currPosition = 0;
double posError = 0;
double prevError = 0;
double cumError = 0;
double rateError = 0;
double rhoKp = 10; //UNITS
double rhoKi = 0.5; //UNITS
double posOutput = 0; 
double rWheel = 0.06985; //meters

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
  setPosition = 0.5; //meters
}

void loop() {
  // put your main code here, to run repeatedly:
  rhoInput = (positionA + positionB)/2;
  rhoInput = rhoInput*rWheel;
  rhoOutput = calcPI(rhoInput);
  digitalWrite(m1dir, LOW);
  digitalWrite(m2dir, HIGH);
  analogWrite(m1pwm, rhoOutput);
  analogWrite(m2pwm, rhoOutput);
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
  //Motor A calculations (left wheel)
  //Calculate Position in rad
  positionA = (countsA*2*PI)/3200; // change 3200 to 64 if encoder counting in encoder counts not motor counts
  
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
  //Motor B calculations (right wheel)
  //Calculate Position in rad
  positionB = (countsB*2*PI)/3200; // change 3200 to 64 if encoder counting in encoder counts not motor counts
  
  //Calculate velocity in rad/s
  velocityB = (positionB - prevPositionB)/(ISRtimeB - prevISRtimeB); //in rad/ms
  velocityB = velocityB*1000; //in rad/s
  prevPositionB = positionB; //set previous position
  prevISRtimeB = ISRtimeB;
}
double calcPI(double input)
{
  
  Serial.print(input);
  Serial.print("\t");
  Serial.print(posError);
  Serial.print("\t");
  currTime = millis();               //Found this link: https://microcontrollerslab.com/pid-controller-implementation-using-arduino/
  elapsedTime = currTime - prevTime; //shouldn't this be elapsedTime += currTime ? This would be the bounds on the integral 
  posError = setPosition - input; //This is the integrand
  cumError += posError*elapsedTime; //This is the full integral - A sum of the error over the time elapsed
  rateError = (posError - prevError)/elapsedTime;
  posOutput = rhoKp*posError + rhoKi*cumError; //V bar calculated by Kp times error plus Ki times integral of error
  posOutput = 255*(posOutput/16); //Turn the voltage to a PWM signal FOR 12 VOLTS - motor data sheet says 12V motor, however battery is only 8(ish) volts 
  Serial.print(cumError);
  Serial.print("\t");
  Serial.print(posOutput);
  Serial.println("");
  if (posOutput > 127)
  {
    posOutput = 127;
  }
  prevError = posError;
  prevTime = currTime;

  return(posOutput);
}

  
