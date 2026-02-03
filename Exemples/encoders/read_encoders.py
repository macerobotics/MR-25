######################################################
#  
#  http://www.macerobotics.com
#  Date : 03/02/2026
#  Version : 0.1
# 
#
#  MIT Licence
######################################################

#!/usr/bin/python
import MR25
import time, sys

print("Reset encodeur : ")
MR25.encoderReset()

while True :
  coderRight = MR25.encoderRight()
  coderLeft = MR25.encoderLeft()
  print("Encoder Right =", coderRight)
  print("Encoder Left =", coderLeft)
  time.sleep(2)
#End of file