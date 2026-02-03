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

while True :
  distance1 = MR25.proxSensor(1) # read sensors 1
  print("Distance capteur 1 =", distance1)
  time.sleep(2)
#End of file