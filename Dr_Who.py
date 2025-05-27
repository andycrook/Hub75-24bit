# Elite Clock

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

    file="/fonts/Frontier-10c.mfont"
    #file="/fonts/Trek-12.mfont"
    matrix.font = matrix.load_minifont(file)

    #print(get_time())
    clock_x=19
    clock_y=61


    count = 0
    start_time = time.ticks_ms()  # or time.ticks_us() for more precision

    #matrix.load_bmp("/3D_models/cobra_texture.bmp",buffered=1)

    mesh= matrix.load_obj("/3D_models/TARDIS40.obj")
    
    mesh.texture_width,mesh.texture_height,mesh.texture=matrix.load_bmp("/3D_models/tardis_master.bmp",0,0, gamma=1.6, brightness=1.7, contrast=1.0, return_data=1)

    #print(mesh.emitters)
    #print(mesh.anim)
    
    
    #mesh= matrix.load_obj("/3D_models/python.obj")
    #mesh.texture_width,mesh.texture_height,mesh.texture=matrix.load_bmp("/3D_models/python.bmp",0,0, gamma=1.2, brightness=1.0, hue=0,contrast=1.0, return_data=1)
    box=[2,2,10] # width, height , depth
    waypoints=40
    #print("anim try")
    #print(mesh.anim)
    mesh.anim[0]=mesh.generate_anim(box,waypoints) # generate an animation and put it in anim slot 0
    #print(a)
    #print(mesh.anim)
    
    #print("anim done")
    mesh.zsort=False
    mesh.light_color = (500,500,400)  # warm orange
    #mesh.light_color = (500,10,10)  # warm orange
    mesh.ambient_color = (3,3,20)  # deep blue ambient glow
    mesh.light_dir = (1,1,-1)  # X Y Z

    mesh.position[2]+=3.0
    mesh.position[1]+=1.0
            # these are the boundaries for pixel drawing
    matrix.VIEWPORT_X = 0
    matrix.VIEWPORT_Y = 0
    matrix.VIEWPORT_XMAX = 64
    matrix.VIEWPORT_YMAX = 64

    percent=0
    frame=0
    #print("starting loops")
    
    #print(mesh.emitters)
    #mesh.init_particles(mesh.emitters)
    #print(mesh.particles)
    try:
       
        while True:
            count += 1
            if count % 100 == 0:
                now = time.ticks_ms()  # or time.ticks_us()
                elapsed = time.ticks_diff(now, start_time)
                print("Loop", elapsed/100, "ms")
                start_time = now  # reset for next 20 loops
            #time.sleep(0.01)
                
            matrix.load_frame("/bin/frame_0"+str(zero_pad(frame))+".bin")
            frame+=1
            if frame>14:
                frame=0
            #
            
            time.sleep(0.02)
            #matrix.show_bmp(0,10,10)
            #matrix.fill(0,0,40)
            #matrix.hline(10,10,20,250,0,0,0,0,64,64)
            #matrix.vline(10,10,8,250,250,0,0,0,64,64)
            #matrix.box(20,20,10,4,0,255,0,0,0,0,64,64)
            #matrix.line(50,50,55,62,0,0,255,0,0,64,64)
            #matrix.ellipse(40,40,10,6,255,0,255,0, 0,0,64,64)
            
            
            
            mesh.rotation[0]+=0.21
            mesh.rotation[1]+=0.01
            #mesh.rotation[2]+=0.05
            percent+=0.1
            if percent>100:
                percent=0
            
            mesh.animate(percent,(0,0,1,0,0,0))
            mesh.move()
            mesh.shading=3
            mesh.render_mode=0
            mesh.wireframe_color=(255,255,255)
            
            #print("drawing obj")
#             matrix.VIEWPORT_X = 1
#             matrix.VIEWPORT_Y = 1
#             matrix.VIEWPORT_XMAX = 63
#             matrix.VIEWPORT_YMAX = 49
            matrix.draw_object(mesh)
            mesh.update_particles(mesh.emitters,mesh.rotation,mesh.position,frame)
            #mesh.draw_particles(mesh.particles,4):
            matrix.draw_particles(mesh.particles,mesh.scale)
            
            
            
            tt=get_time() #about 2ms
            
            matrix.draw_text(clock_x,clock_y,tt[0],(255,255,255),(0,0,0),background_mode=0,buffer=0,shadow=1)
            
            matrix.draw_text(clock_x+17,clock_y,tt[1],(255,255,255),(0,0,0),background_mode=0,buffer=0,shadow=1)
            
            #matrix.draw_text(clock_x+34,clock_y-13,tt[2],(255,255,20),(0,0,0),background_mode=0,buffer=0,shadow=0)
            
            matrix.draw_text(clock_x+32,clock_y,tt[2],(255,255,0),(0,0,0),background_mode=0,buffer=0,shadow=0)
               


            matrix.refresh()

    except KeyboardInterrupt:
      print("Loop interrupted gracefully.")


if __name__ == "__main__":
    main()

