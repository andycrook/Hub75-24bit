from hub75 import Hub75, OBJ3D
import time
import random
import math
import os
import gc

from machine import I2C, Pin, SPI







matrix=Hub75()


matrix.VIEWPORT_X = 1
matrix.VIEWPORT_Y = 1
matrix.VIEWPORT_XMAX = 63
matrix.VIEWPORT_YMAX = 49


#matrix.blend_frames("/bin/Elite_Screens_MASTER.bin","/bin/frame_012.bin",255,'add')
#matrix.refresh()
#print(0/0)


# matrix.load_frame("star trek1.bin")
# matrix.refresh()
# time.sleep(10)
# 


for img in range(0,21):
    print("/bin/nebula"+str(img)+".bin")
    matrix.load_frame("/bin/nebula"+str(img)+".bin")
    matrix.refresh()
    time.sleep(1)


for img in range(0,5):
    matrix.load_frame("/bin/elite_proc"+str(img)+".bin")
    matrix.refresh()
    time.sleep(1)













# #matrix.box(0,49,64,15,0,0,0,1)
# #matrix.box(0,0,63,63,0,0,0,0)
# stars=[]
# for x in range(0,64):
#     for y in range(0,64):
#         col=matrix.get_pixel(x,y)
#         if col[0]+col[1]+col[2]>0:
#             stars.append((col[0],col[1],col[2]))
#             
# print(stars)
# 
# #matrix.refresh()
# for f in range(0,500):
#     rx1=random.randint(matrix.VIEWPORT_X,matrix.VIEWPORT_XMAX)
#     ry1=random.randint(matrix.VIEWPORT_Y,matrix.VIEWPORT_YMAX)
#     rx2=random.randint(matrix.VIEWPORT_X,matrix.VIEWPORT_XMAX)
#     ry2=random.randint(matrix.VIEWPORT_Y,matrix.VIEWPORT_YMAX)
# 
#     col=matrix.get_pixel(rx1,ry1)
#     col2=matrix.get_pixel(rx2,ry2)
#     matrix.set_pixel(rx1,ry1,col2[0],col2[1],col2[2],0,0,64,64)
#     matrix.set_pixel(rx2,ry2,col[0],col[1],col[2],0,0,64,64)

            
stars=[(24, 23, 31), (1, 1, 1), (3, 2, 2), (1, 0, 0), (3, 4, 5), (1, 1, 1), (2, 1, 1), (1, 0, 0), (3, 2, 1), (1, 0, 0), (1, 0, 1), (113, 113, 109), (4, 2, 1), (1, 1, 1), (11, 5, 3), (2, 2, 1), (41, 41, 43), (9, 9, 5), (1, 0, 0), (1, 1, 1), (1, 1, 0), (0, 1, 0), (1, 1, 1), (1, 0, 0), (3, 3, 2), (2, 1, 2), (2, 1, 1), (5, 5, 5), (1, 1, 0), (1, 0, 0), (2, 1, 0), (1, 1, 1), (1, 0, 0), (1, 1, 1), (27, 16, 8), (3, 2, 1), (9, 7, 3), (98, 69, 53), (1, 1, 0), (2, 2, 2), (1, 0, 0), (22, 13, 3), (2, 2, 2), (2, 1, 0), (1, 0, 0), (49, 24, 12), (2, 1, 1), (1, 1, 1), (3, 2, 3), (1, 1, 0), (1, 1, 0), (9, 9, 15), (2, 1, 1), (1, 1, 0), (1, 1, 1), (25, 14, 6), (1, 1, 1), (1, 1, 1), (3, 3, 4), (1, 1, 0), (1, 1, 1), (1, 1, 1), (217, 209, 229), (3, 1, 1), (19, 18, 16), (11, 10, 11), (6, 6, 6), (2, 2, 1), (5, 2, 1), (2, 2, 1), (2, 2, 2), (2, 2, 2)]

for img in range(0,12):
    
    matrix.load_frame("/bin/Elite_Screens_MASTER.bin")


    for f in range(0,3):
        for star in stars:
            rx1=random.randint(matrix.VIEWPORT_X,matrix.VIEWPORT_XMAX)
            ry1=random.randint(matrix.VIEWPORT_Y,matrix.VIEWPORT_YMAX)
            matrix.set_pixel(rx1,ry1,star[0],star[1],star[2],matrix.VIEWPORT_X,matrix.VIEWPORT_Y,matrix.VIEWPORT_XMAX,matrix.VIEWPORT_YMAX)

    if random.randint(0,2)==1:
        filename1="neb_1.bmp"
        matrix.load_bmp(filename1, 0,0, 2.2,(random.randint(2,8)/10),1.0,0,hue=random.randint(0,360),blendmode='alpha')    # gamma of 2.2 to better emulate real colors on an LED matrix




    filename1="Blue_Sun_32_BMP3.bmp"
    #matrix.load_bmp(filename1, 0, 0, 2.2,1.0,1.0,0,scale=0.5)    # gamma of 2.2 to better emulate real colors on an LED matrix
    #matrix.refresh()
    for f in range(random.randint(0,3)):
        matrix.load_bmp(filename1, random.randint(0,74)-34, random.randint(0,74)-34, 2.2,1.0,1.0,0,scale=(random.randint(1,100)/100),hue=random.randint(0,360),blendmode='alpha')    # gamma of 2.2 to better emulate real colors on an LED matrix
    matrix.save_frame("/bin/elite_proc"+str(img)+".bin")
    print("saved","/bin/elite_proc"+str(img)+".bin")
    matrix.refresh()

    
