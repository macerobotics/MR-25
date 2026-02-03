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
  distances = MR25.proxSensorAll()
  print("Distance capteurs =", distances)
  time.sleep(2)
#End of file