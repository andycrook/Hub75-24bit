import network
import socket
import time
import struct
import time as time_up
#from hub75 import Hub75
import random
from ds1307 import DS1307
from machine import I2C, Pin

import math




# DS1307 on 0x68
I2C_ADDR = 0x68     # DEC 104, HEX 0x68

# define custom I2C interface, default is 'I2C(0)'
# check the docs of your device for further details and pin infos
# this are the pins for the Raspberry Pi Pico adapter board
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=800000)
ds1307 = DS1307(addr=I2C_ADDR, i2c=i2c)


# set the RTC module clock...
#ds1307.datetime = (2022, 12, 18, 18, 9, 17, 6)


NTP_DELTA = 2208988800
host = "pool.ntp.org"

led = Pin("LED", Pin.OUT)

ssid = 'SSID'
password = 'YOURPASSWORD'


def set_time():
    NTP_QUERY = bytearray(48)
    NTP_QUERY[0] = 0x1B
    addr = socket.getaddrinfo(host, 123)[0][-1]
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.settimeout(1)
        res = s.sendto(NTP_QUERY, addr)
        msg = s.recv(48)
    finally:
        s.close()
    val = struct.unpack("!I", msg[40:44])[0]
    t = val - NTP_DELTA    
    tm = time.gmtime(t)
    
    t_offset=0
    if is_dst_europe(tm[0], tm[1], tm[2]):
        t_offset = 1  # DST offset
        print("DST OFFSET")
    
    
    
    machine.RTC().datetime((tm[0], tm[1], tm[2], tm[6] + 1, tm[3]+t_offset, tm[4], tm[5], 0)) # tm3 = hours
    
    #ds1307.datetime = (2022, 12, 18, 18, 9, 17, 6)
    
    
    
    
    
def is_dst_europe(year, month, day):
    # DST starts last Sunday of March, ends last Sunday of October
    def last_sunday(month):
        for day in range(31, 24, -1):
            if time.gmtime(time.mktime((year, month, day, 0, 0, 0, 0, 0)))[:3] == (year, month, day):
                if time.gmtime(time.mktime((year, month, day, 0, 0, 0, 0, 0)))[6] == 6:
                    return day
        return 31

    start = last_sunday(3)
    end = last_sunday(10)

    if month > 3 and month < 10:
        return True
    if month == 3 and day >= start:
        return True
    if month == 10 and day < end:
        return True
    return False

    
    
    
    
    
    
    
    
    
    
    
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(ssid, password)

max_wait = 10
while max_wait > 0:
    if wlan.status() < 0 or wlan.status() >= 3:
        break
    max_wait -= 1
    print('waiting for connection...')
    time.sleep(1)

if wlan.status() != 3:
    raise RuntimeError('network connection failed')
else:
    print('connected')
    status = wlan.ifconfig()
    print( 'ip = ' + status[0] )

led.on()
set_time()
print("TIME")
print(time.localtime())
ds1307.datetime =time.localtime()
led.off()