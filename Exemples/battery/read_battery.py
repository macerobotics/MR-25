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
  voltage = MR25.battery()
  print("Tension batterie =", voltage)
  current = MR25.batteryCurrent()
  print("Courant batterie en mA =", current)
  time.sleep(2)

#End of file