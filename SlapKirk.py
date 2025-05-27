# Slap Kirk Clock

from hub75 import Hub75, OBJ3D
import time
import random
import math

import gc
from ds1307 import DS1307
from machine import I2C, Pin, SPI

matrix = Hub75() # Use default pins


I2C_ADDR = 0x68     # DEC 104, HEX 0x68
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=800000)
ds1307 = DS1307(addr=I2C_ADDR, i2c=i2c)

@micropython.native
def get_time():
    
    hour = zero_pad(ds1307.hour)
    minute = zero_pad(ds1307.minute)
    second = zero_pad(ds1307.second)
    
    
   
    return (hour,minute,second)

def zero_pad(num: int) -> str:
    return f"{num:02}"


@micropython.native
def main():

    #file="/fonts/Frontier-10c.mfont"
    file="/fonts/Trek-12.mfont"
    matrix.font = matrix.load_minifont(file)


    clock_x=10
    clock_y=61


    count = 0
    start_time = time.ticks_ms()  # or time.ticks_us() for more precision

    # these are the boundaries for pixel drawing
    matrix.VIEWPORT_X = 0
    matrix.VIEWPORT_Y = 0
    matrix.VIEWPORT_XMAX = 64
    matrix.VIEWPORT_YMAX = 64

    percent=0
    frame=0
    try:
       
        while True:
            count += 1
            if count % 100 == 0:
                now = time.ticks_ms()  # or time.ticks_us()
                elapsed = time.ticks_diff(now, start_time)
                print("Loop", elapsed/100, "ms")
                start_time = now  # reset for next 20 loops
 
                
            matrix.load_frame("/bin/kirkslap_frame_0"+str(zero_pad(frame))+".bin")
            frame+=1
            if frame>29:
                frame=0
            #
            
            
            
            
            time.sleep(0.02)
      
            tt=get_time() #about 2ms
            
            matrix.box(10,49,45,13,0,0,0,1)
            matrix.box(9,48,47,15,255,255,255,0)
            
            matrix.draw_text(clock_x,clock_y,tt[0],(230,190,20),(0,0,0),background_mode=2,buffer=0,shadow=0)
            
            matrix.draw_text(clock_x+16,clock_y,tt[1],(230,190,20),(0,0,0),background_mode=2,buffer=0,shadow=0)
            
            matrix.draw_text(clock_x+32,clock_y,tt[2],(230,190,20),(0,0,0),background_mode=2,buffer=0,shadow=0)
               


            matrix.refresh()

    except KeyboardInterrupt:
      print("Loop interrupted gracefully.")


if __name__ == "__main__":
    main()


