import random
import time
import gc

from machine import I2C, Pin, SPI
import sys
import sdcard
import os
# Define pins for SD card on SPI0
sck=18
mosi=19
miso=16
cs=17


#set up SPI
spi=SPI(0,baudrate=40000000,sck=Pin(sck),mosi=Pin(mosi),miso=Pin(miso))
sd=sdcard.SDCard(spi,Pin(cs))
# Create a instance of MicroPython Unix-like Virtual File System (VFS),
vfs=os.VfsFat(sd)
 
# Mount the SD card
os.mount(sd,'/sd')

print(os.listdir('/sd'))
