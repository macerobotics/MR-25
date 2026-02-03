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

print("Exemple robot MR-25")

while True:
  print("Forward")
  MR25.forward(25)
  time.sleep(2)
  MR25.stop()
  time.sleep(2)
#End of file