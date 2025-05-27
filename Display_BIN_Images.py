from hub75 import Hub75
import time
import os
import random
import time
import gc

from machine import I2C, Pin, SPI
import sys
import sdcard

# Define pins for SD card on SPI0
sck=18
mosi=19
miso=16
cs=17

display = Hub75()

spi=SPI(0,baudrate=40000000,sck=Pin(sck),mosi=Pin(mosi),miso=Pin(miso))
sd=sdcard.SDCard(spi,Pin(cs))
# Create a instance of MicroPython Unix-like Virtual File System (VFS),
vfs=os.VfsFat(sd)
 
# Mount the SD card
os.mount(sd,'/sd')
print(os.listdir('/sd'))

def display_all_bin():

    bin_dir = '/sd/bin'
    
    # Find all bin files in the directory
    for file in os.listdir(bin_dir):
        if file.lower().endswith(".bin"):
            bin_path = bin_dir + "/" +file
            print("Image:",bin_path)
            display.load_frame(bin_path)
            display.refresh()
            time.sleep(2)
            
display_all_bin()