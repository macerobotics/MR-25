// copyleft 

#include <Wire.h>
#include <Arduino.h>
#include "robot.h"
#include "positionControl.h"
#include "controlRobot.h"


// gain PID
#define GAIN_P (float)(0.1) // Distance
#define GAIN_D (float)(0.1) // GAIN_DERIVER_DISTANCE 
#define GAIN_PO (float)(0.11) // GAIN_PROPORTIONNEL_ROTATION
#define GAIN_DO (float)(0.2) // GAIN_DERIVER_ROTATION

#define MAX_SPEED_ASV 30

bool b_controleEnable = false;// activation de l'asservissement.
bool b_controleEnd = false;// indicateur de fin de l'asservissement.

static float consigneDistance = 0;
static float consigneOrientation = 0;
static float error_distance=999999;
static float error_orientation=999999;
static unsigned int count_led = 0;

// Fonction appélé périodiquement
// Gestion du controle du robot (position, asservissement)
bool IRQ_controlRobot(struct repeating_timer *t)
{
  // calcul de position du robot
  positionControl_manage();

  // si l'asservissement est activé.
  if(b_controleEnable == true)
  {
    // asservisement du robot en distance et orientation
    controlRobot_manage();
  }

  if(count_led > 10)
  {
    // clignotement led interne pico
    digitalWrite(25, !digitalRead(25));
    count_led = 0;
  }
  else
  {
    count_led = count_led + 1;
  }

  return true; 
}








void controlRobot_manage(void)
{
float stepDistance, stepOrientation;
float commandDistance, command_orientaton, wheelRightCommand, wheelLeftCommand;
float commandD, commandO;
float speedRobot;


if((b_controleEnable == true)&&((abs(error_distance) > 100)||(abs(error_orientation) > 100)))// erreur acceptable
{

  
  stepDistance = (encoderRight() + encoderLeft())/2;

  // calcul de la vitesse du robot
  speedRobot = (speedwheelRight() + speedwheelLeft())/2;
  
  stepOrientation = encoderRight() - encoderLeft();
  Serial.println(stepOrientation);
  // erreur distance
  error_distance = consigneDistance - stepDistance;

  // erreur orientation
  error_orientation = consigneOrientation - stepOrientation;

  // Commande en distance
  commandD = error_distance*GAIN_P;

  commandDistance = commandD - GAIN_D*speedRobot;

  // Commande en orientation
  commandO = error_orientation*GAIN_PO;
  command_orientaton = commandO - - GAIN_DO*speedRobot;;

  // commande des roues droites et gauches
  wheelRightCommand = commandDistance + command_orientaton;
  wheelLeftCommand = commandDistance - command_orientaton;

  // saturation (limité la commmande)
  wheelRightCommand = constrain(wheelRightCommand, -MAX_SPEED_ASV, MAX_SPEED_ASV);

  // saturation
  wheelLeftCommand = constrain(wheelLeftCommand, -MAX_SPEED_ASV,MAX_SPEED_ASV);

  if(abs(wheelRightCommand) < 15)
    wheelRightCommand = 0;

  if(abs(wheelLeftCommand) < 15)
    wheelLeftCommand = 0;

  //Serial.println("command_orientaton: " + String(command_orientaton));
  //Serial.println("error_orientation: " + String(error_orientation));
  //Serial.println("error_distance: " + String(error_distance));
  
  if(wheelRightCommand > 0)
    motorRight(1,wheelRightCommand);
  else
    motorRight(0,abs(wheelRightCommand));      
    
  if(wheelLeftCommand > 0)
    motorLeft(1,wheelLeftCommand);
  else
    motorLeft(0,abs(wheelLeftCommand));

    
}
else
{
  Serial.println("b_controleEnd true");
  b_controleEnd = true;

  // arret des moteurs
  stopRobot();
}


}

void controlRobot_enable(bool activation)
{
  b_controleEnable = activation;
  error_distance=999999;
  b_controleEnd = false;
}

void controlRobot_WriteCommandeDistance(float consigne)
{
  consigneDistance= consigne;
}

void controlRobot_WriteCommandeOrientation(float consigne)
{
    consigneOrientation= consigne;
}

bool controlRobot_state(void)
{
  return(b_controleEnd);
}



// End of file
