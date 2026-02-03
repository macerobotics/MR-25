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
  MR25.buzzer(600, 2000) # 600 Hz pendant 2 secondes
  time.sleep(2)
  MR25.buzzer(800, 1000) # 800 Hz pendant 1 seconde
  time.sleep(2)
  MR25.buzzer(2000, 500) # 2000 Hz pendant 0.5 seconde
  time.sleep(2)
#End of file

#End of file