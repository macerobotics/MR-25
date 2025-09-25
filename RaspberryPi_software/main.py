#!/usr/bin/python
import MR25
import time, sys

while True:
  MR25.forwardmm(-100)
  time.sleep(4)
  MR25.turnAngle(90)
  time.sleep(4)
  print("Stop")
#End of file