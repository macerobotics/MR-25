// Robot MR-25
// Auteur : Mace Robotics (Adapté Arduino)
// copyleft (🄯) 

#include <Arduino.h>
#include "robot.h"
#include "proxSensors.h"
#include <VL6180X.h>
#include "pio_encoder.h"
#include "controlRobot.h"
#include "positionControl.h"

// ─────────────────────────────────────────────
//  CONSTANTES DE CALIBRATION
// ─────────────────────────────────────────────
// Offset appliqué à la PWM pour compenser les différences mécaniques entre les moteurs.
// Ajuster ces valeurs si le robot ne va pas droit en avançant.
int CALIB_MOTOR_RIGHT = 0; // Compensation moteur droit (en % de PWM)
int CALIB_MOTOR_LEFT  = 0;  // Compensation moteur gauche (en % de PWM)

// ─────────────────────────────────────────────
//  ENCODEURS
// ─────────────────────────────────────────────
// Encodeur 1 (roue droite) : connecté sur la broche GPIO 0
PioEncoder encoder1(0); 
// Encodeur 2 (roue gauche) : connecté sur la broche GPIO 6
PioEncoder encoder2(6); 


// ─────────────────────────────────────────────
//  INITIALISATION DU ROBOT
// ─────────────────────────────────────────────
void init_robot(void)
{
  // --- LED RGB ---
  // GP18 = Rouge, GP20 = Bleu, GP21 = Vert
  pinMode(18, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);
  digitalWrite(18, LOW);  // Rouge éteint
  digitalWrite(20, LOW);  // Bleu éteint
  digitalWrite(21, LOW);  // Vert éteint

  // --- Moteurs ---
  // GP10 = direction moteur droit,  GP11 = PWM moteur droit
  // GP14 = direction moteur gauche, GP15 = PWM moteur gauche
  pinMode(11, OUTPUT);
  pinMode(15, OUTPUT); 
  pinMode(10, OUTPUT);
  pinMode(14, OUTPUT);
  // Résolution PWM à 16 bits (valeurs de 0 à 65535)
  analogWriteResolution(16);

  // Initialisation des capteurs de proximité
  init_proxSensors();

  // Démarrage des encodeurs
  encoder1.begin();
  encoder2.begin();

  // --- Buzzer ---
  pinMode(pin_BUZZER, OUTPUT);
}


// ─────────────────────────────────────────────
//  DÉPLACEMENTS
// ─────────────────────────────────────────────

// Avancer en ligne droite à la vitesse donnée (val = 0..100 %)
void forwardRobot(int val)
{
  motorRight(1, val);
  motorLeft(1, val);
}

// Période de la boucle d'attente de fin de consigne (en µs, non utilisée directement)
int period = 10000;
unsigned long time_now = 0;

// Avancer d'une distance précise en millimètres
// La distance est convertie en ticks d'encodeur, puis confiée à l'asservissement.
void forwardmm(int distance) 
{
  float distTicks;

  // Remise à zéro des encodeurs avant le déplacement
  encoderReset();

  // Conversion mm → ticks : 1 tour de roue = 100.48 mm (diamètre 32 mm × π)
  distTicks = (distance * ENCODER_RESOLUTION) / 100.48;
  
  // Activation de l'asservissement avec la consigne de distance
  controlRobot_enable(true);
  controlRobot_WriteCommandeDistance(distTicks);
  controlRobot_WriteCommandeOrientation(0); // Pas de rotation : avancer droit
  
  Serial.println("Forward mm: " + String(distance));

  time_now = millis();
  // Attente bloquante jusqu'à ce que l'asservissement signale la fin de la consigne
  do {
  } while (controlRobot_state() != true);

  // Désactivation de l'asservissement une fois la consigne atteinte
  controlRobot_enable(false);
}

// Tourner d'un angle précis en degrés (positif = sens horaire)
// L'angle est converti en ticks d'encodeur différentiel.
void turnAngle(int angle)
{
  float angleTicks;

  // Remise à zéro des encodeurs avant la rotation
  encoderReset();

  // Conversion degrés → ticks : 90° correspond à ENCODER_RESOLUTION × 2 ticks d'écart
  angleTicks = (angle * ENCODER_RESOLUTION * 2) / 90;
  
  // Activation de l'asservissement avec la consigne d'orientation
  controlRobot_enable(true);
  controlRobot_WriteCommandeDistance(0);      // Pas de déplacement en ligne droite
  controlRobot_WriteCommandeOrientation(angleTicks);
  
  Serial.println("turnAngle: " + String(angle));

  time_now = millis();
  // Attente bloquante jusqu'à la fin de la rotation
  do {
  } while (controlRobot_state() != true);

  controlRobot_enable(false);
}

// Reculer à la vitesse donnée (val = 0..100 %)
void backRobot(int val) 
{
  motorRight(0, val); // Direction = 0 → marche arrière
  motorLeft(0, val);
}

// Arrêt immédiat : coupe toutes les commandes moteurs (direction + PWM à 0)
void stopRobot()
{
  digitalWrite(10, LOW); // Direction moteur droit à 0
  digitalWrite(11, LOW); // PWM moteur droit à 0
  digitalWrite(15, LOW); // PWM moteur gauche à 0
  digitalWrite(14, LOW); // Direction moteur gauche à 0
}

// Rotation sur place vers la droite (roue droite arrière, roue gauche avant)
void turnRight(int vitesse)
{
  motorRight(0, vitesse);
  motorLeft(1, vitesse);
}

// Rotation sur place vers la gauche (roue droite avant, roue gauche arrière)
void turnLeft(int vitesse)
{
  motorRight(1, vitesse);
  motorLeft(0, vitesse);
}


// ─────────────────────────────────────────────
//  COMMANDES MOTEURS
// ─────────────────────────────────────────────

// Commande du moteur droit
// dir : 1 = avant, 0 = arrière
// pwm : vitesse en pourcentage (0..100), converti en valeur 16 bits (0..65535)
void motorRight(int dir, int pwm) 
{
  float temp;

  // Application de la direction via la broche de sens
  if (dir == 1)
    digitalWrite(10, HIGH);
  else
    digitalWrite(10, LOW);

  // Application de l'offset de calibration
  pwm = pwm + CALIB_MOTOR_RIGHT;
    
  // Sécurité : on n'applique la PWM que si la valeur reste dans les 100 %
  if (pwm <= 100)
  {
    temp = (float)((pwm * 65535) / 100); // Mise à l'échelle 0-100 % → 0-65535
    analogWrite(11, int(temp));
  }
}

// Commande du moteur gauche
// dir : 1 = avant, 0 = arrière
// pwm : vitesse en pourcentage (0..100), converti en valeur 16 bits (0..65535)
void motorLeft(int dir, int pwm)
{
  float temp;

  // Application de la direction via la broche de sens
  if (dir == 1)
    digitalWrite(14, HIGH);
  else
    digitalWrite(14, LOW);

  // Application de l'offset de calibration
  pwm = pwm + CALIB_MOTOR_LEFT;
  
  // Sécurité : on n'applique la PWM que si la valeur reste dans les 100 %
  if (pwm <= 100)
  {
    temp = (float)((pwm * 65535) / 100); // Mise à l'échelle 0-100 % → 0-65535
    analogWrite(15, int(temp));
  }
}


// ─────────────────────────────────────────────
//  LED RGB
// ─────────────────────────────────────────────

// Contrôle de la LED RGB (chaque canal : true = allumé, false = éteint)
// r → GP18, g → GP21, b → GP20
void ledRgb(bool r, bool g, bool b)
{
  digitalWrite(18, r ? HIGH : LOW); // Canal rouge
  digitalWrite(21, g ? HIGH : LOW); // Canal vert
  digitalWrite(20, b ? HIGH : LOW); // Canal bleu
}


// ─────────────────────────────────────────────
//  ENCODEURS
// ─────────────────────────────────────────────

// Retourne le nombre de ticks de l'encodeur gauche (valeur négée pour correspondre au sens avant)
int encoderLeft(void)
{
  return(-encoder2.getCount());
}

// Retourne le nombre de ticks de l'encodeur droit
int encoderRight(void)
{
  return(encoder1.getCount());     
}

// Remet les deux compteurs d'encodeurs à zéro
void encoderReset(void)
{
  encoder1.reset();  
  encoder2.reset();     
}


// ─────────────────────────────────────────────
//  ODOMÉTRIE
// ─────────────────────────────────────────────

// Calcule et retourne l'orientation actuelle du robot en degrés.
// Basée sur la différence de ticks entre les deux encodeurs (odométrie différentielle).
float orientationRobot(void)
{
  int stepOrientation, angleDegree;

  // Différence de ticks : si les roues ont parcouru des distances différentes, le robot a tourné
  stepOrientation = encoderRight() - encoderLeft();

  // Conversion ticks → degrés : 90° = ENCODER_RESOLUTION × 2 ticks d'écart
  angleDegree = (stepOrientation * 90) / (ENCODER_RESOLUTION * 2);

  return(angleDegree);
}

// ─────────────────────────────────────────────
//  CALIBRATION DES MOTEURS
// ─────────────────────────────────────────────

void calibration_motorRight(int motorCalibR)
{
  CALIB_MOTOR_RIGHT = motorCalibR;
}


void calibration_motorLeft(int motorCalibL)
{
  CALIB_MOTOR_LEFT = motorCalibL;
}

// End of file
