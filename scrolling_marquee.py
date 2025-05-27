from hub75 import Hub75, OBJ3D
import time
import random
import math
import os
import gc

from machine import I2C, Pin, SPI



matrix=Hub75()

file="/fonts/Frontier-10c.mfont"

matrix.font = matrix.load_minifont(file)


#matrix.draw_text(text="Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur.",marquee=True)
# This is about the limit without running out of memory
matrix.draw_text(text="CROOK 178.25 +1.32  |  ANDR 192.01 -0.52  |  B5 14.32 +0.87  |  TNG 905.44 +3.21  |  TOS 421.19 -1.12  |  MOON 182.55 +2.14  |",marquee=True)


speed=1



try:
   
    while True:
        matrix.load_frame("/bin/BLACK.bin",0)
        matrix.scrolling_marquee(60,speed,(255,255,255),(0,0,0),VIEWPORT=(0,0,63,64))

        matrix.refresh()
        time.sleep(0.05)
    
except KeyboardInterrupt:
  print("Loop interrupted gracefully.")
