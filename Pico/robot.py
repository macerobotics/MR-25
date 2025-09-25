# Author : Mace Robotics (www.macerobotics.com)
# Firmware du Robot MR-25
# Version : 0.46
# Date : 20/05/25
# MIT Licence


from machine import Pin, Timer, PWM
import encoder
import vl6180x
import machine
import time
import math 
import select
import sys
import os

# pour la reception usb
poll_obj = select.poll()
poll_obj.register(sys.stdin,1)



# CONSTANTES
PERIMETER_ROBOT_MM = 273 # Robot perimeter (mm)
ENCODEURS_RESOLUTION = 1024 # tick pour 1 tour de roue.
PERIMETER_WHEEL = 100 # wheels perimeter (mm)

#GAIN_P = 0.7
GAIN_P = 1
GAIN_PO = 1 #pid orientation
# FIN CONSTANTES


# 1200 ticks = 90°
# 900 ticks = 100 mm

# Variable globales
deltaDistance = 0
old_stepDistance = 0
stepDistance = 0
stepOrientation = 0
XposRobot = 0
YposRobot = 0
controleEnable = False
command_distance = 0
command_orientation = 0
wheelRightCommand = 0
wheelLeftCommand = 0
max_speed_control = 0
error_distance = 9999999
error_orientation = 9999999
error_asv = 9999999
b_endControle = False
TypeControleAsv = 0 # 1 = Distance, 2 = Orientation, 0 = nothing
# Fin variables globales


# encoder init
codeurRight = encoder.Encoder(1,0)
codeurLeft = encoder.Encoder(6,7)

# I2C
i2c = machine.I2C(0, scl=machine.Pin(13, Pin.PULL_UP), sda=machine.Pin(12, Pin.PULL_UP), freq=400000)

i2c_ina219 = machine.I2C(1, scl=machine.Pin(27, Pin.PULL_UP), sda=machine.Pin(26, Pin.PULL_UP), freq=400000)


# prox sensors
xshut1 = Pin(9, Pin.OUT, Pin.PULL_UP)
xshut2 = Pin(8, Pin.OUT, Pin.PULL_UP) 
xshut3 = Pin(4, Pin.OUT, Pin.PULL_UP)
xshut4 = Pin(3, Pin.OUT, Pin.PULL_UP)
xshut5 = Pin(2, Pin.OUT, Pin.PULL_UP)
xshut1.value(0)
xshut2.value(0)
xshut3.value(0)
xshut4.value(0)
xshut5.value(0)



# init led RGB
rgb_red = Pin(18, Pin.OUT)   
rgb_blue = Pin(20, Pin.OUT)      
rgb_green = Pin(21, Pin.OUT)

# motors
pwm1 = PWM(Pin(11))
pwm2 = PWM(Pin(15))  
dir1 = Pin(10, Pin.OUT)
dir2 = Pin(14, Pin.OUT)
pwm1.freq(100)
pwm2.freq(100)
dir1.value(0)
dir2.value(0)
pwm1.duty_u16(0)
pwm2.duty_u16(0)


# battery measure
batt = machine.ADC(28) # 12 bits

# buzzer
buzz = PWM(Pin(19, mode=Pin.OUT))

ledPico = Pin(25, Pin.OUT)


time.sleep(3)
#####################################################  
#####################################################
#####################################################

# read the battery tension
def battery():
  value = batt.read_u16()
  tension = (value * 3.3)/65535
  return(tension*2)

def proxRead(sensor):
  xshut1.value(0)
  xshut2.value(0)
  xshut3.value(0)
  xshut4.value(0)
  xshut5.value(0)
  
  sensor = int(sensor) # conversion en int
  
  if (sensor == 1):
    xshut1.value(1)
  elif (sensor == 2):
    xshut2.value(1)
  elif (sensor == 3):
    xshut3.value(1)
  elif (sensor == 4):
    xshut4.value(1)  
  elif (sensor == 5):
    xshut5.value(1)
    
  try:
    proxSensor1 = vl6180x.Sensor(i2c, 0x29)
    return (proxSensor1.range())
  except:
    return (-1)

def proxReadALS(sensor):
  xshut1.value(0)
  xshut2.value(0)
  xshut3.value(0)
  xshut4.value(0)
  xshut5.value(0)
  
  sensor = int(sensor) # conversion en int
  
  if (sensor == 1):
    xshut1.value(1)
  elif (sensor == 2):
    xshut2.value(1)
  elif (sensor == 3):
    xshut3.value(1)
  elif (sensor == 4):
    xshut4.value(1)  
  elif (sensor == 5):
    xshut5.value(1)
    
  try:
    proxSensor1 = vl6180x.Sensor(i2c, 0x29)
    return (proxSensor1.get_als(1))
  except:
    return (-1) 

# read the right encoder
def encoderRight():
  return(codeurRight.read())

# read the left encoder
def encoderLeft():
  return(codeurLeft.read())

# reset encoder counter
def encoderReset():
  codeurLeft.reset()
  codeurRight.reset()
  
def ledRgb(red, green, blue):
  rgb_red.value(red)
  rgb_blue.value(blue)
  rgb_green.value(green)
  
# buzzer
def buzzer(frequence, duty):
  buzz.freq(frequence)
  buzz.duty_u16(int(duty))
  
def buzzerStop():
  buzz.duty_u16(int(0))
  
# direction (0 ou 1), speped (0 à 100)
def motorRight(direction, speed):
  dir1.value(direction)
  pwm1.duty_u16(int((speed/100)*65_535))

# direction (0 ou 1), speped (0 à 100)
def motorLeft(direction, speed):
  dir2.value(direction)
  pwm2.duty_u16(int((speed/100)*65_535))
  
def forward(speed):
  motorRight(1,speed)
  motorLeft(1,speed)
      
def back(speed):
  motorRight(0,speed)
  motorLeft(0,speed)

def turnRight(speed):
  motorRight(0,speed)
  motorLeft(1,speed)

def turnLeft(speed):
  motorRight(1,speed)
  motorLeft(0,speed)
  
def stop():
  pwm1.duty_u16(0)
  pwm2.duty_u16(0)
      
def brake():
  pwm1.duty_u16(65025)
  pwm2.duty_u16(65025)
  
def positionX():
  global XposRobot
  return((XposRobot*100)/1024)

def positionY():
  global YposRobot
  return(YposRobot)
  
def orientation():
  stepOrientation = encoderRight() - encoderLeft()
  print("stepOrientation:", stepOrientation)
  angle = (stepOrientation * 90)/(696)
  return(angle)

# forward with control
def forwardmm(distance, speed):
  global controleEnable, command_distance, max_speed_control, command_orientation, b_endControle, error_distance
  global TypeControleAsv, error_asv

  if (abs(distance) > 10):
      encoderReset()
      TypeControleAsv = 1
      controleEnable = True
      b_endControle = False
      error_asv = 9999999
      command_distance = (distance*900)/100 #conversion en tick
      command_orientation = 0
      max_speed_control = speed
      
      while (b_endControle != True):
        time.sleep(1)

  
def turnAngle(angle, speed):
  global controleEnable, command_distance, max_speed_control, command_orientation, b_endControle, error_distance
  global TypeControleAsv, error_asv 
  
  encoderReset()
  TypeControleAsv = 2
  controleEnable = True
  b_endControle = False
  error_asv = 99999
  command_distance = 0 
  command_orientation = (-angle*1200)/90
  max_speed_control = speed

  while (b_endControle != True):
    time.sleep(1)
  controleEnable = False
  print("End turnAngle")
  
################################################################
################################################################
################################################################
################################################################
################################################################
################################################################
# INTERRUPTION

timer=Timer()
timer1=Timer()
timer2=Timer()
timer3=Timer()


counter_controlRobot = 0

def controlRobot(timer1):
  global command_distance, command_orientation, stepDistance, controleEnable, wheelRightCommand, wheelLeftCommand
  global stepOrientation, error_distance, b_endControle, error_orientation
  global error_asv,TypeControleAsv, counter_controlRobot
  
  counter_controlRobot = counter_controlRobot + 1


  #if ((abs(error_distance) > 10) or (abs(error_orientation) > 10)) :
  if (abs(error_asv) > 10) :
    if controleEnable == True:
            
          # erreur distance
          error_distance = command_distance - stepDistance
          
          # erreur orientation
          error_orientation = command_orientation - stepOrientation    
          
          if (TypeControleAsv == 1):
            error_asv = error_distance
          elif (TypeControleAsv == 2):
            error_asv = error_orientation
            
            
          command = error_distance*GAIN_P
          
          command_orientaton = error_orientation*GAIN_PO
          
          
          # wheels command
          wheelRightCommand = command + command_orientaton
          wheelLeftCommand = command - command_orientaton
    
    
    
    
          # saturation
          if(wheelRightCommand > 50):
              wheelRightCommand = 50
              
          # saturation            
          if(wheelLeftCommand > 50):
              wheelLeftCommand = 50
              
          # saturation
          if(wheelRightCommand < -50):
              wheelRightCommand = -50
              
          # saturation            
          if(wheelLeftCommand < -50):
              wheelLeftCommand = -50
              
          
          if(wheelRightCommand > 0):
              motorRight(1,wheelRightCommand)
          else:
              motorRight(0,abs(wheelRightCommand))      
    
          if(wheelLeftCommand > 0):
              motorLeft(1,wheelLeftCommand)
          else:
              motorLeft(0,abs(wheelLeftCommand))
              
              
          
  else:
    #print("Fin CONTROLE2")
    if controleEnable == True:
      stop()
    b_endControle = True



# Position du robot (x, y et orientation)
def positionControl(timer3):
  global old_stepDistance, XposRobot, YposRobot, stepOrientation, stepDistance, stepOrientation
  

  
  stepDistance = (encoderRight() + encoderLeft())/2
  stepOrientation = encoderRight() - encoderLeft()
  
  deltaDistance = stepDistance - old_stepDistance
  old_stepDistance = stepDistance
  stepOrientation = encoderRight() - encoderLeft()
  radOrientation = stepOrientation*(0.0011)
  
  # delat X and Y position robot calculator
  deltaX = deltaDistance*math.cos(radOrientation)
  deltaY = deltaDistance*math.sin(radOrientation)
  
  # X and Y position of the robot
  XposRobot = XposRobot + deltaX
  YposRobot = YposRobot + deltaY
  
  Orientation = math.degrees(radOrientation)

  #print("stepOrientation :", stepOrientation)
  #print("Orientation :", Orientation)

# Gestion de la led de la carte Raspberry Pico et de la tension batterie
def gestion_led_pi(timer2):
  if(battery() > 3.7):
    ledPico.toggle()
    f = open('_log.txt', 'a')
    f.write(str(battery()) + " " +str(proxRead(1)))
    f.write('\n')
    f.close()
  else:
    ledPico.on() # tension de batterie trop basse
    
# Gestion reception commande debug
def receive_USB_command(timer):   
  if poll_obj.poll(0):
    # lecture data récu
    data = sys.stdin.read(1)
    # Si reception =
    if (data =='#'):
      print ("Battery = " , battery())
      print ("Capteur 1 = " , proxRead(1))
      print ("Capteur 2 = " , proxRead(2))
      print ("Capteur 3 = " , proxRead(3))
      print ("Capteur 4 = " , proxRead(4))
      print ("Capteur 5 = " , proxRead(5))  
####################################################################################
####################################################################################  
  
# timer.init(freq=10, mode=Timer.PERIODIC, callback=tick)
timer.init(freq=10, callback=receive_USB_command) # forme minimale

timer1.init(freq=100, callback=controlRobot) # forme minimale

# timer.init(freq=10, mode=Timer.PERIODIC, callback=tick)
timer2.init(freq=1.5, callback=gestion_led_pi) # forme minimale

timer3.init(freq=100, callback=positionControl) # gestion led pico
################################################################
# End of file