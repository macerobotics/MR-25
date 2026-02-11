// copyleft 


#include <Arduino.h>
#include "robot.h"
#include "proxSensors.h"
#include <VL6180X.h>
#include "pio_encoder.h"
#include "controlRobot.h"
#include "positionControl.h"

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
  motorRight(1,val);
  motorLeft(1,val);
}


int period = 10000;
unsigned long time_now = 0;


// distance en millimètres
void forwardmm(int distance) 
{
float distTicks;

  // reset des encodeurs
  encoderReset();

  // conversion des millimètres en ticks
  distTicks = (distance*ENCODER_RESOLUTION)/100.48; // 100.48 mm= 1 tour de roue (roue de 32 mm)
  
  controlRobot_enable(true);

  controlRobot_WriteCommandeDistance(distTicks);
  controlRobot_WriteCommandeOrientation(0);
  
  Serial.println("Forward mm: " + String(distance));

  time_now = millis();

  do{
    // attendre la fin de la consigne
  }while(controlRobot_state() != true);

  controlRobot_enable(false);

}

// angle en degrés
void turnAngle(int angle)
{
float angleTicks;

  // reset des encodeurs
  encoderReset();

  // conversion des degre en ticks
  angleTicks = (angle*ENCODER_RESOLUTION*2)/90; // 100.48 mm= 1 tour de roue (roue de 32 mm)


  
  controlRobot_enable(true);

  controlRobot_WriteCommandeDistance(0);
  controlRobot_WriteCommandeOrientation(angleTicks);
  
  Serial.println("turnAngle mm: " + String(angle));

  time_now = millis();

  do{
    // attendre la fin de la consigne
  }while(controlRobot_state() != true);

  controlRobot_enable(false);
}

void backRobot(int val) 
{
  motorRight(0,val);
  motorLeft(0,val);
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
  motorRight(0,vitesse);
  motorLeft(1,vitesse);
}

void turnLeft(int vitesse)
{
  motorRight(1,vitesse);
  motorLeft(0,vitesse);
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
  return(-encoder2.getCount());
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

  angleDegree = (stepOrientation * 90)/(ENCODER_RESOLUTION*2);
  return(angleDegree);
}

// End of file
