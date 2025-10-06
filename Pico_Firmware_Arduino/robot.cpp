


#include <Arduino.h>
#include "robot.h"
#include "proxSensors.h"
#include <VL6180X.h>
#include "pio_encoder.h"

PioEncoder encoder1(0); 
PioEncoder encoder2(6); 

void init_robot(void)
{
  // led RGB
  pinMode(18, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);

  digitalWrite(18, LOW);  
  digitalWrite(20, LOW);  
  digitalWrite(21, LOW);  

  // motors
  pinMode(11, OUTPUT);
  pinMode(15, OUTPUT); 
  pinMode(10, OUTPUT);
  pinMode(14, OUTPUT);

  analogWriteResolution(16);

  init_proxSensors();

  encoder1.begin();
  encoder2.begin();

  // buzzer
  pinMode(pin_BUZZER, OUTPUT);


}




void forwardRobot(int val)
{
  motorRight(val, 1);
  motorLeft(val, 1);
}

void forwardmm(int distance) 
{
  Serial.println("Forward mm: " + String(distance));
}

void turnAngle(int angle)
{
  Serial.println("Turn angle: " + String(angle));
}

void backRobot(int val) 
{
  motorRight(val, 0);
  motorLeft(val, 0);
}

void stopRobot()
{
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);

  digitalWrite(15, LOW);
  digitalWrite(14, LOW);
}

void turnRight(int vitesse)
{
  motorRight(vitesse, 0);
  motorLeft(vitesse, 1);
}

void turnLeft(int vitesse)
{
  motorRight(vitesse, 1);
  motorLeft(vitesse, 0);
}

void motorRight(int dir, int pwm) 
{
float temp;

  if(dir == 1)
    digitalWrite(10, HIGH);
  else
    digitalWrite(10, LOW);
  
  temp = (float)((pwm*65535)/100);
  analogWrite(11, int(temp));
}

void motorLeft(int dir, int pwm)
{
float temp;

  if(dir == 1)
    digitalWrite(14, HIGH);
  else
    digitalWrite(14, LOW);
  
  temp = (float)((pwm*65535)/100);
  analogWrite(15, int(temp));
}

void ledRgb(bool r, bool g, bool b)
{
  if(r == 1)
    digitalWrite(18, HIGH);
  else
    digitalWrite(18, LOW);

    
  if(g == 1)
    digitalWrite(21, HIGH);
  else
    digitalWrite(21, LOW);

  if(b == 1)
    digitalWrite(20, HIGH);
  else
    digitalWrite(20, LOW);
}


int encoderLeft(void)
{
  return(encoder2.getCount());
}

int encoderRight(void)
{
  return(encoder1.getCount());     
}

void encoderReset(void)
{
  encoder1.reset();  
  encoder2.reset();     
}

float orientationRobot(void)
{
int stepOrientation, angleDegree;

  stepOrientation = encoderRight() - encoderLeft();
  
  angleDegree = (stepOrientation * 90)/(696);
  return(angleDegree);
}

// End of file
