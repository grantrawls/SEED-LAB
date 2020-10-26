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
int maxCtrl = 90; //max PWM signal
int minCtrl = 63; //minimum PWM signal to move robot consistently forward
const int period = 20;  
int currPosTime = 0; 
int prevPosTime = 0;
int deltaPosTime = 0;

double choosePosition = 0.3048; //meters
double setPosition = 0;
double currPosition = 0;
double posError = 0;
double prevPosError = 0;
double totalPosError = 0;
double ratePosError = 0;
double deltaPosError = 0;
double prevPosTotalError = 0;

double chooseAngle = 2*PI; //radians
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

double rhoKp = 10; //UNITS //10
double rhoKi = 0.5; //UNITS //0.05
double posOutput = 0; 
double rWheel = 0.06985; //meters
double dWheels = 0.269875; //meters

double phiKp = 10; //UNITS //10
double phiKi = 0.5; //UNITS //0.05
double phiInput = 0;
double angleOutput = 0; 
bool goForward = 0;
bool notGoneForward = 1;
bool runFirst = 1;
bool runSecond = 1;

int tempCountsA = 0;
int tempCountsB = 0;
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
}

void loop() {
  // put your main code here, to run repeatedly:
  
  //phiCalcPI(chooseAngle);
        if(goForward == 1)
      {
       if (runFirst == 1)
       {
        Serial.println("GOING FORWARDS");
        tempCountsA = countsA;//
        tempCountsB = countsB;//
        countsA = 0;//
        countsB = 0;//
        prevPosTime = currAngleTime;//
        runFirst = 0;
       }
        rhoCalcPI(0.3048);

        if (runSecond == 0)
        {
        goForward = 0;//
        countsA = tempCountsA;//
        countsB = tempCountsB;//
        runSecond = 1;
        }
      }else
      {
        phiCalcPI(chooseAngle);
      }
  //rhoCalcPI(choosePosition);
  //calculate Va and Vb from Vbar and deltaV
  //change PWM conversion here

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
  positionA = (countsA*2*PI)/2950; // change 3200 to 64 if encoder counting in encoder counts not motor counts
  
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
  positionB = (countsB*2*PI)/2950; // change 3200 to 64 if encoder counting in encoder counts not motor counts
  
  //Calculate velocity in rad/s
  velocityB = (positionB - prevPositionB)/(ISRtimeB - prevISRtimeB); //in rad/ms
  velocityB = velocityB*1000; //in rad/s
  prevPositionB = positionB; //set previous position
  prevISRtimeB = ISRtimeB;
}

void rhoCalcPI(double setPosition)
{
  
  //Set robot to go forward
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

    if (posOutput > maxCtrl)
    {
      posOutput = maxCtrl;
    }else if(posOutput < minCtrl)
    {
      posOutput = minCtrl;
    }if(posError > -0.01 && posError < 0.01)
    {
      posOutput = rhoKp*posError + rhoKi*deltaPosError*period; //V bar (Volts)
      runSecond = 0;
      goForward = 0;
    }
    prevPosError = posError;
    prevPosTime = currPosTime;
    prevPosTotalError = totalPosError;
  }
    Serial.print(rhoInput);
    Serial.print("\t");
    Serial.print(posError);
    Serial.print("\t");
    Serial.print(totalPosError);
    Serial.print("\t");
    Serial.print(posOutput);
    Serial.print("\t");
    Serial.print(countsA);
    Serial.print("\t");
    Serial.print(countsB);
    Serial.println("");
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

  //fix this for phi controller (if statements)
  /*if(countsA > 14000 && countsB > 14000)
  {
    setAngle = setAngle - phiInput;
    countsA = 0;
    countsB = 0;
  }*/
  

  if(deltaAngleTime >= period)
  {
    angleError = setAngle - phiInput;
    totalAngleError += angleError; 
    deltaAngleError = angleError - prevAngleError;
    angleOutput = phiKp*angleError + phiKi*deltaAngleError*period; //V bar (Volts)
    angleOutput = (256*(angleOutput/24)) - 1; //PWM signal change for 12 volt motor (battery at 8 though?) and split in half because two wheels.

    if (angleOutput > maxCtrl)
    {
      angleOutput = maxCtrl;
    }else if(angleOutput < minCtrl)
    {
      angleOutput = minCtrl;
    }if(angleError > -0.01 && angleError < 0.01)
    {
      angleOutput = phiKp*angleError + phiKi*deltaAngleError*period; //V bar (Volts)
      if(goForward == 0)
      {
        if(notGoneForward == 1)
        {
          goForward = 1;
          notGoneForward = 0;
        }
      }
    }
    prevAngleError = angleError;
    prevAngleTime = currAngleTime;
    prevAngleTotalError = totalAngleError;
  }
    Serial.print(phiInput);
    Serial.print("\t");
    Serial.print(angleError);
    Serial.print("\t");
    Serial.print(totalAngleError);
    Serial.print("\t");
    Serial.print(angleOutput);
    Serial.print("\t");
    Serial.print(countsA);
    Serial.print("\t");
    Serial.print(countsB);
    Serial.println("");
  analogWrite(m1pwm, angleOutput); //motor A input
  analogWrite(m2pwm, angleOutput); //motor B input
}


  
