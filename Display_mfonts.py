from hub75 import Hub75, OBJ3D
import time
import random
import math
import os
import gc

from machine import I2C, Pin, SPI



matrix=Hub75()

font_dir = '/fonts'
delete_broken_fonts=True

for file in os.listdir(font_dir):
    if file.lower().endswith(".mfont"):
        font_path = font_dir + "/" +file
        print(font_path)
        try:
            matrix.font = matrix.load_minifont(font_path)
            codes = sorted(matrix.font["glyphs"].keys())
            print("GLYPHS in font:",len(codes))
            if len(codes)==0 and delete_broken_fonts:
                os.remove(font_path)
                continue
            char=""
            for code in codes:
                char = char+chr(code)
            print(char)
            matrix.pixel_buffer=[]
            matrix.load_frame("/bin/BLACK.bin",0)
            
            color=(255,255,255)
            BGcolor=(0,0,10)
            
            matrix.hline(0,15,64,0,255,0)
            matrix.draw_text(0,15, "Font test:", color,BGcolor,background_mode=0,buffer=2,shadow=0)
            
            matrix.hline(0,35,64,0,255,0)
            matrix.draw_text(0,35, file[:-6], color,BGcolor,background_mode=0,buffer=2,shadow=0)
            
            matrix.hline(0,55,64,0,255,0)
            matrix.draw_text(0,55, "08badgqpyA^{}jz", color,BGcolor,background_mode=0,buffer=2,shadow=0)
            
            matrix.refresh()

            time.sleep(2)
               
        except KeyboardInterrupt:
            print("Loop interrupted gracefully.")


