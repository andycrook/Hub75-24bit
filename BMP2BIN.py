import os
from hub75 import Hub75
import random
import time
import gc
from ds1307 import DS1307
from machine import I2C, Pin
from machine import I2C, Pin, SPI
import sys
import sdcard
import os
# 
# # Define pins for SD card on SPI0
# sck=18
# mosi=19
# miso=16
# cs=17
# 
# 
# #set up SPI
# spi=SPI(0,baudrate=40000000,sck=Pin(sck),mosi=Pin(mosi),miso=Pin(miso))
# sd=sdcard.SDCard(spi,Pin(cs))
# # Create a instance of MicroPython Unix-like Virtual File System (VFS),
# vfs=os.VfsFat(sd)
#  
# # Mount the SD card
# os.mount(sd,'/sd')

#print(os.listdir('/sd'))
display = Hub75()


def file_exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False

def convert_all_bmps(only_new):

    bmp_dir = '/bmp'
    bin_dir = '/bin'


    # Ensure the bin directory exists
    try:
        os.mkdir(bin_dir)
    except OSError:
        pass  # Directory already exists
    
    # Find all BMP files in the directory
        for file in os.listdir(bmp_dir):
            if file.lower().endswith(".bmp"):
                bmp_path = bmp_dir + "/" + file
                bin_name = file.rsplit(".", 1)[0] + ".bin"
                bin_path = bin_dir + "/" + bin_name
                if only_new:
                    if not file_exists(bin_path):
                        convert(bmp_path, bin_path)
                else:
                    convert(bmp_path, bin_path)
                    
def convert(filename1,filename2):

    print(f"Processing: {filename1} -> {filename2}")
    
    # Load and save using the provided functions gamma brightness contrast
    print("Loading BMP...")
    display.load_bmp(filename1, 0, 0, 2.2,1.0,1.0,0.0)    # gamma of 2.2 to better emulate real colors on an LED matrix
    
    print("Saving framebuffer...")
    display.save_frame(filename2)
    print("Display image....")
    display.refresh()
    print(f"Saved: {filename2}")

# Example usage:
#convert('image_file.bmp','/bin/image_file.bin')

convert_all_bmps(True)
