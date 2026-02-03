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
  MR25.forward(25)
  time.sleep(2)
  
  MR25.back(25)
  time.sleep(2)
  
  MR25.turnRight(50)
  time.sleep(2)
  
  MR25.turnLeft(50)
  time.sleep(2)
  
  MR25.stop()
  time.sleep(2)
#End of file