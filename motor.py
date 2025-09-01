#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Date : 19/09/2022
# Version : 0.1
# Auteur : Mace Robotics
# 
# Lib motor for MR-Pico 
#
#
# MOTOR1 : GP11, GP10



from machine import Pin, PWM


a = (-100/65025)

class Motor:
    
  def __init__(self, pin1, pin2):
      self.pwm1 = PWM(Pin(pin1))
      self.pwm2 = PWM(Pin(pin2))
      self.pwm1.freq(20000)
      self.pwm2.freq(20000)
      
  def forward(self, speed):
      speed = (speed - 100)/(a)
      self.pwm1.duty_u16(65025)
      self.pwm2.duty_u16(int(speed))
      
  def reverse(self, speed):
      self.pwm2.duty_u16(65025)
      self.pwm1.duty_u16(speed)
      
  def stop(self):
      self.pwm1.duty_u16(0)
      self.pwm2.duty_u16(0)
      
  def brake():
      self.pwm1.duty_u16(65025)
      self.pwm2.duty_u16(65025)  
# end of file