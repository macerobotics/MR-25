
// Robot MR-25
// Auteur : Mace Robotics (Adapté Arduino)
// Version : 0.44
// copyleft (🄯) 

#include <Arduino.h>
#include <Wire.h>
//#include <arduino-timer.h>


#define TIMER_INTERRUPT_DEBUG         1
#define _TIMERINTERRUPT_LOGLEVEL_     2



#include "RPi_Pico_TimerInterrupt.h"
#include "robot.h"
#include "proxSensors.h"
#include "positionControl.h"
#include "controlRobot.h"



RPI_PICO_Timer ITimer1(1);

//auto timer = timer_create_default(); // create a timer with default settings

// Constantes
const float VERSION_FIRMWARE = 0.44;

// Variables globales
bool send_data = false;
String data_send = "";
String cmd_recu = "";
String parametre1_recu = "";
int parametre2_recu = 0;
bool deux_para = false;

// LED interne (GP25)
const int LED_PIN = 25;
const int LED_LOW_BAT = 22;

// UART (UART0 → Serial1 sur Pico)
#define RX_PIN 17
#define TX_PIN 16



#define TIMER1_INTERVAL_MS    100 // millisecondes




void setup() 
{
  // Initialisation UART
  Serial1.setTX(TX_PIN);
  
  Serial1.setRX(RX_PIN);
  
  Serial1.begin(230400);
  
  // Debug via USB
  Serial.begin(115200);

  delay(2000);

  Serial.println("Robot MR-25 Start");


  pinMode(LED_PIN, OUTPUT);

  // LED low batterie
  pinMode(LED_LOW_BAT, OUTPUT);
  digitalWrite(LED_LOW_BAT, LOW);

  init_robot();
  
   ledRgb(0,0,0);

   
  // Buzzer de démarrage

  tone (pin_BUZZER, 600);
  delay(500);
  noTone(pin_BUZZER);

  // init du timer pour l'asservissement.
  ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS * 1000, IRQ_controlRobot);






}

// Fonction pour traiter une commande reçue
void traiterCommande(String data)
{
  cmd_recu = "";
  parametre1_recu = "";
  parametre2_recu = 0;
  deux_para = false;

  //Serial.println("Recu: " + data);

  // Exemple : #MF,100!
  if (data.startsWith("#") && data.endsWith("!")) {
    int virguleIndex = data.indexOf(',');
    int exclIndex = data.indexOf('!');

    if (virguleIndex > 0) {
      cmd_recu = data.substring(1, virguleIndex);

      String reste = data.substring(virguleIndex + 1, exclIndex);
      int sep = reste.indexOf(',');

      if (sep >= 0) {
        deux_para = true;
        parametre1_recu = reste.substring(0, sep);
        parametre2_recu = reste.substring(sep + 1).toInt();
      } else {
        parametre1_recu = reste;
      }
    } else {
      // commande simple sans paramètre (ex : #FV!)
      cmd_recu = data.substring(1, exclIndex);
    }

  Serial.println("cmd_recu: " +cmd_recu);
  Serial.println("parametre1_recu: " +parametre1_recu);
  Serial.println("parametre2_recu: " +String(parametre2_recu));  

  
    // Exécution des commandes
    if (cmd_recu == "FV") {
      data_send = String(VERSION_FIRMWARE);
      send_data = true;
    }

    else if (cmd_recu == "PR") {
      data_send = String(readProx(parametre1_recu.toInt()));
      send_data = true;
    }

    else if (cmd_recu == "MF") {
      forwardRobot(parametre1_recu.toInt());
    }

    else if (cmd_recu == "FM") {
      forwardmm(parametre1_recu.toInt());
    }

    else if (cmd_recu == "TA") {
      turnAngle(parametre1_recu.toInt());
    }

    else if (cmd_recu == "MB") {
      backRobot(parametre1_recu.toInt());
    }

    else if (cmd_recu == "STP") {
      send_data = false;
      stopRobot();
    }

    else if (cmd_recu == "TR") {
      turnRight(parametre1_recu.toInt());
    }

    else if (cmd_recu == "TL") {
      turnLeft(parametre1_recu.toInt());
    }

    else if (cmd_recu == "MOTR") {
      motorRight(parametre1_recu.toInt(), parametre2_recu);
    }

    else if (cmd_recu == "MOTL") {
      motorLeft(parametre1_recu.toInt(), parametre2_recu);
    }

    else if (cmd_recu == "RGB") {
      int val = parametre1_recu.toInt();
      bool r = val / 100 % 10;
      bool g = val / 10 % 10;
      bool b = val % 10;
      ledRgb(r, g, b);
    }

    else if (cmd_recu == "LEDB") {
      send_data = false;
      if(parametre1_recu.toInt() == 1)
        digitalWrite(LED_LOW_BAT, HIGH);
      else
        digitalWrite(LED_LOW_BAT, LOW);
    }

    // read left encoder
    else if (cmd_recu == "EDL") {
      data_send = String(encoderLeft());
      send_data = true;
    }

    // read right encoder
    else if (cmd_recu == "EDR") {
      data_send = String(encoderRight());
      send_data = true;
    }

    // reset encoder
    else if (cmd_recu == "ERZ") {
      encoderReset();
      send_data = false;
    }

    // read position X
    else if (cmd_recu == "POX") {
      data_send = String(positionRobotX());
      send_data = true;
    }

    // read position Y
    else if (cmd_recu == "POY") {
      data_send = String(positionRobotY());
      send_data = true;
    }

    // read speed wheels right
    else if (cmd_recu == "SWR") {
      data_send = String(speedwheelRight());
      send_data = true;
    }

    // read speed wheels left
    else if (cmd_recu == "SWL") {
      data_send = String(speedwheelLeft());
      send_data = true;
    }

    else if (cmd_recu == "BUZ") {
      tone(pin_BUZZER, parametre1_recu.toInt(), parametre2_recu);
    }
    
  }
}

void loop() {
  // Lecture UART
  if (Serial1.available()) {
    String input = Serial1.readStringUntil('\n');
    input.trim();
    traiterCommande(input);
  }

    // Lecture UART (DEBUG)
  /*if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    Serial.println(input);
    input.trim();
    traiterCommande(input);
  }*/

  // Envoi UART
  if (send_data) {
    Serial1.println(data_send);
    Serial.println(data_send);//DEBUG
    send_data = false;
  }



  
  delay(10); 
}

// End of file
