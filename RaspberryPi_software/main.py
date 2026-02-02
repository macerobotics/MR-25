######################################################
#  Python API
#  This library is used for the MR-25 robot.
#  http://www.macerobotics.com
#  Date : 06/10/2025
#  Version : 0.1
# 
#  Fonctionnne avec Python 3
#
#  MIT Licence

######################################################

#!/usr/bin/python
import MR25
import time, sys

while True : 
  print("Orientation =", MR25.orientation())
  time.sleep(2)

#End of file