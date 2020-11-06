void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void rotateToPhi(double phiSetPosition){
    // rhoInput is the current forward position
  rhoInput = (positionA + positionB)/2;
  rhoInput = rhoInput*rWheel;
  //Current angular position (radians)
  phiDot = (rWheel*(positionA - positionB))/dWheels;
  Serial.print(phiSetPosition);
  Serial.print("\t");
  Serial.print(phiDot);
  Serial.println("");  
  if(phiSetPosition - phiDot <= .05 && phiSetPosition - phiDot >=-.05){
    digitalWrite(m1dir, LOW);
    digitalWrite(m2dir, HIGH);
    analogWrite(m1pwm, 0);
    analogWrite(m2pwm, 0);
    resetCountsA = countsA;
    resetCountsB = countsB;
    if(notGoneforwardsYet == 1){
      goForwards(.3048);
    }
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

double goForwards(double distance){
  //rhoInput is currentPosition
  countsA = 0;
  countsB = 0;
  rhoInput = 0;
  //rhoSetPosition is the goal position
  rhoSetPosition = rhoInput + distance;
  Serial.print("FORWARDS FUNCTION STARTED");
  while(rhoSetPosition - rhoInput >= .02 || rhoSetPosition - rhoInput <=-.02){
  if(rhoSetPosition - rhoInput <= .02 && rhoSetPosition - rhoInput >=-.02){
    digitalWrite(m1dir, LOW);
    digitalWrite(m2dir, HIGH);
    analogWrite(m1pwm, 0);
    analogWrite(m2pwm, 0);  
    } else if(rhoSetPosition - rhoInput >.01){
    digitalWrite(m1dir, LOW);
    digitalWrite(m2dir, HIGH);
    analogWrite(m1pwm, motorSpeed);
    analogWrite(m2pwm, motorSpeed);
    } else {
    digitalWrite(m1dir, HIGH);
    digitalWrite(m2dir, LOW);
    analogWrite(m1pwm, motorSpeed);
    analogWrite(m2pwm, motorSpeed); 
    }
    Serial.print(rhoInput);
    Serial.print("\t");
    Serial.print(rhoSetPosition);
    Serial.println("");  
    digitalWrite(m1dir, LOW);
    digitalWrite(m2dir, HIGH);
    analogWrite(m1pwm, 0);
    analogWrite(m2pwm, 0);
    rhoInput = (positionA + positionB)/2;
    rhoInput = rhoInput*rWheel;
  }
  notGoneforwardsYet = 0;
 }
