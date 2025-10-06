

#include <Wire.h>
#include <Arduino.h>
#include "robot.h"
#include <VL6180X.h>


VL6180X sensor1;
VL6180X sensor2;
VL6180X sensor3;
VL6180X sensor4;
VL6180X sensor5;

/* List of adresses for each sensor - after reset the address can be configured */
#define address0 0x20
#define address1 0x22
#define address2 0x24
#define address3 0x28
#define address4 0x30
#define RANGE 1

// set the pins to shutdown
#define SHT_LOX1 9
#define SHT_LOX2 8
#define SHT_LOX3 28
#define SHT_LOX4 3
#define SHT_LOX5 2

void init_proxSensors()
{
  
  Wire.setSDA(12);
  Wire.setSCL(13);
  Wire.begin();

  // Reset all connected sensors
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  pinMode(SHT_LOX4, OUTPUT);
  pinMode(SHT_LOX5, OUTPUT);
  digitalWrite(SHT_LOX1, LOW); 
  digitalWrite(SHT_LOX2, LOW); 
  digitalWrite(SHT_LOX3, LOW); 
  digitalWrite(SHT_LOX4, LOW);   
  digitalWrite(SHT_LOX5, LOW); 
  
  delay(100);

  // Sensor1
  //Serial.println("Start Sensor 0");
  digitalWrite(SHT_LOX1, HIGH);
  delay(50);
  sensor1.init();
  sensor1.configureDefault();
  sensor1.setAddress(address0);
  Serial.println(sensor1.readReg(0x212),HEX); // read I2C address
  sensor1.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor1.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor1.setTimeout(500);
  sensor1.stopContinuous();
  sensor1.setScaling(RANGE); // configure range or precision 1, 2 oder 3 mm
  delay(300);
  sensor1.startInterleavedContinuous(100);
  delay(100);

  // Sensor2
  //Serial.println("Start Sensor 1");
  digitalWrite(SHT_LOX2, HIGH);
  delay(50);
  sensor2.init();
  sensor2.configureDefault();
  sensor2.setAddress(address1);
  Serial.println(sensor2.readReg(0x212),HEX);
  sensor2.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor2.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor2.setTimeout(500);
  sensor2.stopContinuous();
  sensor2.setScaling(RANGE);
  delay(300);
  sensor2.startInterleavedContinuous(100);
  delay(100);

  // Sensor3
  //Serial.println("Start Sensor 3");
  digitalWrite(SHT_LOX3, HIGH);
  delay(50);
  sensor3.init();
  sensor3.configureDefault();
  sensor3.setAddress(address2);
  Serial.println(sensor3.readReg(0x212),HEX);
  sensor3.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor3.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor3.setTimeout(500);
  sensor3.stopContinuous();
  sensor3.setScaling(RANGE);
  delay(300);
  sensor3.startInterleavedContinuous(100);
  delay(100);

  // Sensor4
  //Serial.println("Start Sensor 4");
  digitalWrite(SHT_LOX4, HIGH);
  delay(50);
  sensor4.init();
  sensor4.configureDefault();
  sensor4.setAddress(address3);
  Serial.println(sensor4.readReg(0x212),HEX);
  sensor4.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor4.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor4.setTimeout(500);
  sensor4.stopContinuous();
  sensor4.setScaling(RANGE);
  delay(300);
  sensor4.startInterleavedContinuous(100);
  delay(100);

  // Sensor5
  //Serial.println("Start Sensor 5");
  digitalWrite(SHT_LOX5, HIGH);
  delay(50);
  sensor5.init();
  sensor5.configureDefault();
  sensor5.setAddress(address4);
  Serial.println(sensor5.readReg(0x212),HEX);
  sensor5.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor5.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor5.setTimeout(500);
  sensor5.stopContinuous();
  sensor5.setScaling(RANGE);
  delay(300);
  sensor5.startInterleavedContinuous(100);
  delay(100);
  
}

int readProx(int capteur)
{
int distance;

  if(capteur == 1)
    distance = sensor1.readRangeContinuousMillimeters();
  else if(capteur == 2)
    distance = sensor2.readRangeContinuousMillimeters();    
  else if(capteur == 3)
    distance = sensor3.readRangeContinuousMillimeters();  
  else if(capteur == 4)
    distance = sensor4.readRangeContinuousMillimeters();  
  else if(capteur == 5)
    distance = sensor5.readRangeContinuousMillimeters();  
  else
    distance = 255;
    
  return distance;
}

// End of file
