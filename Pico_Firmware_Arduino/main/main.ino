// Robot MR-25
// Auteur : Mace Robotics (Adapté Arduino)
// Version : 0.45
// copyleft (🄯) 

#include <Arduino.h>
#include <Wire.h>
//#include <arduino-timer.h>

// Niveau de débogage du timer (0 = désactivé, 1 = activé)
#define TIMER_INTERRUPT_DEBUG         1
// Niveau de verbosité des logs du timer (0 à 4)
#define _TIMERINTERRUPT_LOGLEVEL_     2

#include "RPi_Pico_TimerInterrupt.h"
#include "robot.h"
#include "proxSensors.h"
#include "positionControl.h"
#include "controlRobot.h"

// Instanciation du timer matériel n°1 du Raspberry Pi Pico
RPI_PICO_Timer ITimer1(1);

// Version du firmware
const float VERSION_FIRMWARE = 0.45;

// --- Variables globales de communication série ---
bool send_data = false;       // Indique si une donnée est prête à être envoyée
String data_send = "";        // Donnée à envoyer via UART
String cmd_recu = "";         // Commande extraite de la trame reçue
String parametre1_recu = "";  // Premier paramètre de la commande (chaîne)
int parametre2_recu = 0;      // Deuxième paramètre de la commande (entier)
bool deux_para = false;       // Vrai si la commande contient deux paramètres

// Broche de la LED interne du Pico (GP25)
const int LED_PIN = 25;
// Broche de la LED signalant une batterie faible
const int LED_LOW_BAT = 22;

// Broches UART0 (Serial1 sur Pico)
#define RX_PIN 17
#define TX_PIN 16

// Période d'interruption du timer d'asservissement (en millisecondes)
#define TIMER1_INTERVAL_MS    100


// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────
void setup() 
{
  // Configuration et démarrage de l'UART de communication (Serial1)
  Serial1.setTX(TX_PIN);
  Serial1.setRX(RX_PIN);
  Serial1.begin(230400);
  
  // Démarrage du port série USB pour le débogage
  Serial.begin(115200);

  delay(500);

  Serial.println("Robot MR-25 Start");

  // Configuration de la LED interne en sortie
  pinMode(LED_PIN, OUTPUT);

  // Configuration de la LED batterie faible en sortie, éteinte par défaut
  pinMode(LED_LOW_BAT, OUTPUT);
  digitalWrite(LED_LOW_BAT, LOW);

  // Initialisation du robot (moteurs, capteurs, etc.)
  init_robot();
  
  // Extinction de la LED RGB au démarrage
  ledRgb(0, 0, 0);

  // Bip de démarrage via le buzzer
  tone(pin_BUZZER, 600);
  delay(500);
  noTone(pin_BUZZER);

  // Démarrage du timer d'interruption pour la boucle d'asservissement
  // Fréquence : TIMER1_INTERVAL_MS ms → conversion en µs pour l'API
  ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS * 1000, IRQ_controlRobot);
}


// ─────────────────────────────────────────────
//  TRAITEMENT DES COMMANDES SÉRIE
// ─────────────────────────────────────────────
// Format attendu : #CMD,param1,param2!
//   - Commence par '#', se termine par '!'
//   - Les paramètres sont séparés par des virgules
//   - param2 est optionnel
void traiterCommande(String data)
{
  // Réinitialisation des variables de la trame précédente
  cmd_recu = "";
  parametre1_recu = "";
  parametre2_recu = 0;
  deux_para = false;

  // Vérification de la structure de la trame (#...!)
  if (data.startsWith("#") && data.endsWith("!")) {
    int virguleIndex = data.indexOf(',');
    int exclIndex = data.indexOf('!');

    if (virguleIndex > 0) {
      // Extraction de la commande (entre '#' et la première virgule)
      cmd_recu = data.substring(1, virguleIndex);

      // Extraction du ou des paramètres (entre la virgule et '!')
      String reste = data.substring(virguleIndex + 1, exclIndex);
      int sep = reste.indexOf(',');

      if (sep >= 0) {
        // Deux paramètres détectés : param1 (string) et param2 (int)
        deux_para = true;
        parametre1_recu = reste.substring(0, sep);
        parametre2_recu = reste.substring(sep + 1).toInt();
      } else {
        // Un seul paramètre
        parametre1_recu = reste;
      }
    } else {
      // Commande sans paramètre (ex : #FV!)
      cmd_recu = data.substring(1, exclIndex);
    }

    // Affichage de la commande et des paramètres pour le débogage
    Serial.println("cmd_recu: " + cmd_recu);
    Serial.println("parametre1_recu: " + parametre1_recu);
    Serial.println("parametre2_recu: " + String(parametre2_recu));  

    // ── Exécution des commandes ──────────────────────────────────────

    // FV → Retourne la version du firmware
    if (cmd_recu == "FV") {
      data_send = String(VERSION_FIRMWARE);
      send_data = true;
    }

    // PR → Lecture d'un capteur de proximité (param1 = numéro du capteur)
    else if (cmd_recu == "PR") {
      data_send = String(readProx(parametre1_recu.toInt()));
      send_data = true;
    }

    // MF → Avancer à vitesse donnée (param1 = vitesse en %)
    else if (cmd_recu == "MF") {
      forwardRobot(parametre1_recu.toInt());
    }

    // FM → Avancer d'une distance en millimètres (param1 = distance en mm)
    else if (cmd_recu == "FM") {
      forwardmm(parametre1_recu.toInt());
    }

    // TA → Tourner d'un angle précis (param1 = angle en degrés)
    else if (cmd_recu == "TA") {
      turnAngle(parametre1_recu.toInt());
    }

    // MB → Reculer à vitesse donnée (param1 = vitesse en %)
    else if (cmd_recu == "MB") {
      backRobot(parametre1_recu.toInt());
    }

    // STP → Arrêt immédiat du robot
    else if (cmd_recu == "STP") {
      send_data = false;
      stopRobot();
    }

    // TR → Tourner à droite (param1 = vitesse en %)
    else if (cmd_recu == "TR") {
      turnRight(parametre1_recu.toInt());
    }

    // TL → Tourner à gauche (param1 = vitesse en %)
    else if (cmd_recu == "TL") {
      turnLeft(parametre1_recu.toInt());
    }

    // MOTR → Commande directe du moteur droit (param1 = direction, param2 = vitesse)
    else if (cmd_recu == "MOTR") {
      motorRight(parametre1_recu.toInt(), parametre2_recu);
    }

    // MOTL → Commande directe du moteur gauche (param1 = direction, param2 = vitesse)
    else if (cmd_recu == "MOTL") {
      motorLeft(parametre1_recu.toInt(), parametre2_recu);
    }

    // RGB → Contrôle de la LED RGB (param1 = valeur 3 chiffres ex: 110 = R+G, pas B)
    else if (cmd_recu == "RGB") {
      int val = parametre1_recu.toInt();
      bool r = val / 100 % 10;  // Centaines → rouge
      bool g = val / 10  % 10;  // Dizaines  → vert
      bool b = val       % 10;  // Unités    → bleu
      ledRgb(r, g, b);
    }

    // LEDB → Allumage/extinction de la LED batterie faible (param1 = 1/0)
    else if (cmd_recu == "LEDB") {
      send_data = false;
      if (parametre1_recu.toInt() == 1)
        digitalWrite(LED_LOW_BAT, HIGH);
      else
        digitalWrite(LED_LOW_BAT, LOW);
    }

    // EDL → Lecture de l'encodeur gauche (retourne le nombre de ticks)
    else if (cmd_recu == "EDL") {
      data_send = String(encoderLeft());
      send_data = true;
    }

    // EDR → Lecture de l'encodeur droit (retourne le nombre de ticks)
    else if (cmd_recu == "EDR") {
      data_send = String(encoderRight());
      send_data = true;
    }

    // ERZ → Remise à zéro des deux encodeurs
    else if (cmd_recu == "ERZ") {
      encoderReset();
      send_data = false;
    }

    // CALIBR → calibration du moteurs droit
    else if (cmd_recu == "CALIBR") {
      calibration_motorRight(parametre1_recu.toInt());
      send_data = false;
    }

    // CALIBL → calibration du moteurs gauche
    else if (cmd_recu == "CALIBL") {
      calibration_motorLeft(parametre1_recu.toInt());
      send_data = false;
    }

    // POX → Lecture de la position X du robot (en mm)
    else if (cmd_recu == "POX") {
      data_send = String(positionRobotX());
      send_data = true;
    }

    // POY → Lecture de la position Y du robot (en mm)
    else if (cmd_recu == "POY") {
      data_send = String(positionRobotY());
      send_data = true;
    }

    // SWR → Lecture de la vitesse de la roue droite
    else if (cmd_recu == "SWR") {
      data_send = String(speedwheelRight());
      send_data = true;
    }

    // SWL → Lecture de la vitesse de la roue gauche
    else if (cmd_recu == "SWL") {
      data_send = String(speedwheelLeft());
      send_data = true;
    }

    // BUZ → Émission d'un son (param1 = fréquence en Hz, param2 = durée en ms)
    else if (cmd_recu == "BUZ") {
      tone(pin_BUZZER, parametre1_recu.toInt(), parametre2_recu);
    }
    
  }
}


// ─────────────────────────────────────────────
//  BOUCLE PRINCIPALE
// ─────────────────────────────────────────────
void loop() {

  // Réception et traitement des commandes via UART (communication principale)
  if (Serial1.available()) {
    String input = Serial1.readStringUntil('\n');
    input.trim(); // Suppression des espaces et caractères de fin de ligne
    traiterCommande(input);
  }

  // Réception via USB série pour le débogage (décommentez pour activer)
  /*if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    Serial.println(input);
    input.trim();
    traiterCommande(input);
  }*/

  // Envoi de la réponse via UART si une donnée est en attente
  if (send_data) {
    Serial1.println(data_send);
    Serial.println(data_send); // DEBUG : écho sur le port USB
    send_data = false;
  }

  // Petite pause pour éviter de surcharger le CPU
  delay(10); 
}

// End of file
