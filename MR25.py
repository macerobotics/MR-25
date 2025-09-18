#!/usr/bin/python

######################################################
#  Python API
#  This library is used for the MR-25 robot.
#  http://www.macerobotics.com
#  Date : 19/08/2025
#  Version : 0.7
# 
#  Modif : suppression controleEnable, gerer direct dans lib
#  Fonctionnne avec Python 3
#
#  MIT Licence

######################################################
from math import*

import serial
import time
import os
import INA219




###############################################################
###############################################################

__all__ = ['firmwareVersion']# OK
__all__ = ['battery']# OK
__all__ = ['batteryCurrent']# OK
__all__ = ['proxSensor']
__all__ = ['proxSensorAll']
__all__ = ['forward']# OK
__all__ = ['back']# OK
__all__ = ['stop'] # OK
__all__ = ['turnRight']# OK
__all__ = ['turnLeft']# OK
__all__ = ['motorRight']# OK
__all__ = ['motorLeft']# OK
__all__ = ['encoderLeft']# OK
__all__ = ['encoderRight']# OK
__all__ = ['encoderReset']# OK
__all__ = ['writeCommand']
__all__ = ['readData']
__all__ = ['buzzer']
__all__ = ['buzzerStart']
__all__ = ['buzzerStop']
__all__ = ['ledRGB'] # OK
__all__ = ['ledLowBatt'] # OK
__all__ = ['orientation'] 
###############################################################
###############################################################




# init serial port, baud rate = 230400
port = serial.Serial('/dev/ttyAMA0', 230400,  bytesize=8, parity='N', stopbits=1)
time.sleep(0.5)

ina219 = INA219.INA219(addr=0x41)

# Read firmware version of the microcontroller
def firmwareVersion():
  """
        Read the firmware version (microcontroller)
        Exemple:
        >> firmwareVersion()
  """
  liste = []
  value = 0
  port.flushInput() # reset serial receive buffer
  port.write(b'#FV!\n')
  value = readData()
  return float(value)
  
 
#---------------------------------------------------------------------
#-------------[ MR-25 class sensors methods]--------------------------

# get battery tension (volt unit)
def battery():
  """
        Read battery tension
        return tension (volt)
        Exemple:
        >> battery()
  """
  liste = []
  value = 0
  return (ina219.getBusVoltage_V())
  
# get battery current (mA unit)
def batteryCurrent():
  """
        Read battery current
        return mA
        Exemple:
        >> batteryCurrent()
  """
  liste = []
  value = 0
  return (ina219.getCurrent_mA())


  
# read proximity sensor
def proxSensor(sensor):
  """
        Read proximity sensor
        parameter : 1 to 5
        return proximity sensor (0 to 2000), millimeter unit
        Exemple, read sensor 2:
        >> proxSensor(2)
  """
  if( sensor not in(1,2,3,4,5) ) :
    print ("error parameter")
  else:
    liste = []
    value = 0
    port.flushInput() # reset serial receive buffer
    port.write(b"#PR,")
    sensor = bytes(str(sensor), 'utf-8')
    port.write(sensor)
    port.write(b'!\n')
    value = readData()
    return float(value)

# read all proximity sensor
def proxSensorAll():
  pd = []
  pd.append(proxSensor(1))
  pd.append(proxSensor(2))
  pd.append(proxSensor(3))
  pd.append(proxSensor(4))
  pd.append(proxSensor(5))
  return(pd)
#---------------------------------------------------------------------
#-------------[ MR-25 move robot methods]------------

# the robot move forward
def forward(speed):
  """
        move forward MR-25
        parameter : 0 to 100
        Exemple:
        >> forward(20)
  """
  if speed > -1 and speed < 101:
    speed = str(speed)
    port.write(b"#MF,")
    speed = bytes(speed, 'utf-8')
    port.write(speed)
    port.write(b'!\n')
  else:
    print("error speed value")

# the robot move back
def back(speed):
  """
        move back MR-25
        parameter : 0 to 100
        Exemple:
        >> forward(20)
  """
  if speed > -1 and speed < 101:
    speed = str(speed)
    port.write(b'#MB,')
    speed = bytes(speed, 'utf-8')
    port.write(speed)
    port.write(b'!\n')
  else:
    print("error speed value")
	

# the robot stop
def stop():
  """
        stop the robot
        Exemple:
        >> stop()
  """
  port.write(b'#STP!\n')
 
# the robot turn right
def turnRight(speed):
  """
        turn right
        parameter : speed (0 to 100)
        max speed = 100
        min speed = 0
        Exemple:
        >> turnRight(30)
  """
  if speed > -1 and speed < 101:
    speed = str(speed)
    port.write(b"#TR,")
    speed = bytes(speed, 'utf-8')
    port.write(speed)
    port.write(b'!\n')
  else:
    print("error speed value")

# the robot turn left
def turnLeft(speed):
  """
        turn left
        parameter : speed (0 to 100)
        max speed = 100
        min speed = 0
        Exemple:
        >> turnLeft(30)
  """
  if speed > -1 and speed < 101:
    speed = str(speed)
    port.write(b'#TL,')
    speed = bytes(speed, 'utf-8')
    port.write(speed)
    port.write(b'!\n')
  else:
    print("error speed value")
 
  
# the motor right
def motorRight(direction, speed):
  """
        motor right control
        parameter 1 : direction (0 or 1)
        parameter 2 : speed ( 0 to 100)     
        Exemple:
        >> motorRight(1, 50)
  """
  dir = bytes(str(direction), 'utf-8')
  pwm = bytes(str(speed), 'utf-8')
  port.write(b'#MOTR,')
  port.write(dir)
  port.write(b',')
  port.write(pwm)
  port.write(b'!\n')

# the motor left
def motorLeft(direction, speed):
  """
        motor left control
        parameter 1 : direction (0 or 1)
        parameter 2 : speed ( 0 to 100)     
        Exemple:
        >> motorLeft(1, 50)
  """
  dir = bytes(str(direction), 'utf-8')
  pwm = bytes(str(speed), 'utf-8')
  port.write(b'#MOTL,')
  port.write(dir)
  port.write(b',')
  port.write(pwm)
  port.write(b'!\n')
  

  
########################################
# robot go (X and Y Coordinate),
# speed : speed of the robot (0 to 100)
# cordX : Coordinate axe X (millimeter)
# cordY : Coordinate axe Y (millimeter)

  

  

#---------------------------------------------------------------------
#-------------[ MR-25 buzzer methods]-------------------------

# buzzer control
def buzzer(frequency):
  """
        buzzer control
  """
  if frequency > 0 and frequency < 20001:
    freq = bytes(str(frequency), 'utf-8')
    port.write(b'#BUZ,')
    port.write(freq)
    port.write(b'!\n')
  else:
    print("Error frequency value")
  
# buzzer stop
def buzzerStop():
    port.write(b'#BUZS')
    port.write(b'!\n')

# led RGB (exemple "100", "101",..)
def ledRGB(red_green_blue):
  color = bytes(red_green_blue, 'utf-8')
  print("COLOR", color)
  port.write(b'#RGB,')
  port.write(color)
  port.write(b'!\n')
  
# control low batt led
def ledLowBatt(value):
  if ((value == 1) or (value == 0)):
    value = bytes(str(value), 'utf-8')
    port.write(b'#LEDB,')
    port.write(value)
    port.write(b'!\n')
  else:
    print("erreur")
#---------------------------------------------------------------------
#-------------[ MR-25 encoders robot methods]-------------------------

# the encoderleft
def encoderLeft():
  """
        read the encoder left value  
        Exemple:
        >> encoderLeft()
  """
  liste = []
  value = 0
  port.flushInput() # reset serial receive buffer
  port.write(b'#EDL!\n')
  value = readData()
  return int(value)
  
# the encoderleft
def encoderRight():
  """
        read the encoder right value  
        Exemple:
        >> encoderRight()
  """
  liste = []
  value = 0
  port.flushInput() # reset serial receive buffer
  port.write(b'#EDR!\n')
  value = readData()
  return int(value)
  
# the encoderReset
def encoderReset():
  """
        reset the encoder  
        Exemple:
        >> encoderReset()
  """
  port.write(b'#ERZ')
  port.write(b'!\n')
 
# the orientation
def orientation():
  """
        read the orientation   
        Exemple:
        >> orientation()
  """
  stepOrientation = encoderRight() - encoderLeft()
  print("stepOrientation:", stepOrientation)
  angle = (stepOrientation * 90)/(1000)
  return(angle)

  
#---------------------------------------------------------------------
#-------------[ MR-25 serial2 methods]----------------------------
  
# 
def serial2Write(data):
  """
        serial 2 write 
        Exemple:
        >> serial2Write("HELLO")
  """
  port.write(b"#SRLW,")
  port.write(data)
  port.write(b"!")
  

#------------------------------------------------------------
#-------------[ MR-25 class utils private methods]------------

# check 
def check_speed(speed, distance):
  acceleration = 0.5 # acceleration in the STM32 (trapezoid generator)
  check1 = distance*acceleration
  check2 = speed*speed
  
  if(distance == 0):
    return 1
  
  if check1 > check2:
      return 1
  else:
    print("Error speed to hight !")
    return 0
  

# convert list to unsigned int
def __convListToUint(liste):
  a=''
  i=0
  result=0
  while i < (len(liste)):
    a = a + liste[i]
    i = i + 1
  try:
    result = int(a)
    return(result)
  except ValueError:
    pass
  

# write command
def writeCommand(command):
  port.write(b'#')
  port.write(command)
  port.write(b'!')
  
# read data
def readData():
  chaine = port.readline()
  #print("chaine = ", chaine)
  pos1 = chaine.find(b'$')
  pos2 = chaine.find(b'\n')
  chaine = chaine[pos1+1:pos2]
  return chaine 
  
# convert list to float
def __convListToFloat(liste):
  a=''
  i=0
  while i < (len(liste)):# minus '?'
    a = a + liste[i]
    i = i + 1
  return(float(a))
  
# the robot move forward with control (pas d'attente de la fin du trapeze)
def forwardControl(speed, distance):
  """
        move forward MR-25 with control
  """
  controlEnable()

  if control_robot == True:
    print("Forward with control enable")
    distance = int(distance)
    speed = str(speed)
    distance = str(distance)
    port.write(b"#MFC,")
    port.write(distance)
    port.write(b",")
    port.write(speed)
    port.write(b"!")
    port.flushInput() # reset serial receive buffer
  else:
    print("error : control robot disable")
  
# the robot move forward with control (pas d'attente de la fin du trapeze)
def __backControl(speed, distance):
  """
        move forward MR-25 with control
  """
  controlEnable()

  if control_robot == True:
    print("Forward with control enable")
    distance = int(distance)
    speed = str(speed)
    distance = str(distance)
    port.write(b"#MBC,")
    port.write(distance)
    port.write(b",")
    port.write(speed)
    port.write(b"!")
    port.flushInput() # reset serial receive buffer
  else:
    print("error : control robot disable")

	
# the robot move forward with control (pas d'attente de la fin du trapeze)
def __turnLeftControl(speed, angle):
  """
        move forward MR-25 with control
  """
  controlEnable()

  if control_robot == True:
    print("Forward with control enable")
    angle = int(angle)
    speed = str(speed)
    angle = str(angle)
    port.write(b"#TLC,")
    port.write(angle)
    port.write(b",")
    port.write(speed)
    port.write(b"!")
    port.flushInput() # reset serial receive buffer
  else:
    print("error : control robot disable")


# the robot move forward with control (pas d'attente de la fin du trapeze)
def __turnRightControl(speed, angle):
  """
        move forward MR-25 with control
  """
  controlEnable()

  if control_robot == True:
    print("Forward with control enable")
    angle = int(angle)
    speed = str(speed)
    angle = str(angle)
    port.write(b"#TRC,")
    port.write(angle)
    port.write(b",")
    port.write(speed)
    port.write(b"!")
    port.flushInput() # reset serial receive buffer
  else:
    print("error : control robot disable")


# control robot (postion/orientation disable)
control_robot = False


 
# end file
