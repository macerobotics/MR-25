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

while True:
  MR25.forwardmm(-100)# reculer de 100 mm
  time.sleep(2)
  MR25.turnAngle(90)# tourner de 90°
  time.sleep(1)
#End of file