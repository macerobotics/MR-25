

#include <Wire.h>
#include <Arduino.h>
#include "robot.h"
#include "positionControl.h"

const float pi = 3.14159;

static int XposRobot, YposRobot;
static float vitesse_roueD, vitesse_roueG;

// Calcul de la position du robot
void positionControl_manage()
{
int stepDistance, stepOrientation, deltaDistance;
static int old_stepDistance;
float radOrientation, deltaX, deltaY, Orientation;
float distanceRoueD, distanceRoueG;
static float old_encoderRight, old_encoderLeft;

  // lecture de la vitesse de la roue droite
  distanceRoueD = encoderRight() - old_encoderRight;
  old_encoderRight = encoderRight();

  // lecture de la vitesse de la roue gauche
  distanceRoueG = encoderLeft() - old_encoderLeft;
  old_encoderLeft = encoderLeft();

  // calcul de la vitesse des roues
  vitesse_roueD = distanceRoueD/100;// V = d/t (100ms)

  // calcul de la vitesse des roues
  vitesse_roueG = distanceRoueG/100;// V = d/t

//Serial.println("vitesse_roueD: " + String(vitesse_roueD));
//Serial.println("vitesse_roueG: " + String(vitesse_roueG));


  stepDistance = (encoderRight() + encoderLeft())/2;
  
  stepOrientation = encoderRight() - encoderLeft();
  
  deltaDistance = stepDistance - old_stepDistance;
  old_stepDistance = stepDistance;

  radOrientation = stepOrientation*(0.00024);

  // delat X and Y position robot calculator
  deltaX = deltaDistance*cos(radOrientation);
  deltaY = deltaDistance*sin(radOrientation);

  // X and Y position of the robot
  XposRobot = XposRobot + deltaX;
  YposRobot = YposRobot + deltaY;

  //Serial.println("stepOrientation = "+String(stepOrientation));
  //Serial.println("radOrientation = "+String(radOrientation));

  // Conversion radian -> degrée ((180/pi)
  Orientation = radOrientation* (57.29577);
  //Serial.println("Orientation = "+String(Orientation));
}

// renvoie la vitesse de la roue droite
int speedwheelRight(void)
{
  return(vitesse_roueD);
}

// renvoie la vitesse de la roue gauche
int speedwheelLeft(void)
{
  return(vitesse_roueG);
}


int positionRobotX(void)
{
  // conversion en mm
  return((XposRobot*100)/ENCODER_RESOLUTION);

}

int positionRobotY(void)
{
  // conversion en mm
  return((YposRobot*100)/ENCODER_RESOLUTION);
}

// End of file
