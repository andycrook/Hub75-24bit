from hub75 import Hub75, OBJ3D
import time
import random
import math

import gc
from ds1307 import DS1307
from machine import I2C, Pin, SPI

matrix = Hub75() # Use default pins

file="/fonts/4x6.mfont"
   
matrix.font = matrix.load_minifont(file)


matrix.draw_text(1,6,"4x6 mini font",(255,255,255),(0,0,20),background_mode=0,buffer=2,shadow=1)
matrix.hline(20,20,4,200,0,0)

matrix.refresh()          