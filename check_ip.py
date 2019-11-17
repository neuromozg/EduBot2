#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import socket
import fcntl
import struct
import smbus as I2C

from ina219 import INA219
from ina219 import DeviceRangeError

import Adafruit_SSD1306

from PIL import Image
from PIL import ImageFont
from PIL import ImageDraw

import picamera
import os

#I2C Address of device
EDUBOT_ADDRESS = 0x27
REG_WHY_IAM = 0x00

#для инициализации
SHUNT_OHMS = 0.01
MAX_EXPECTED_AMPS = 2.0

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', bytes(ifname[:15], 'utf-8'))
    )[20:24])

def CheckEduBot():
    res = bus.read_byte_data(EDUBOT_ADDRESS, REG_WHY_IAM)
    return (res == 0X2A) #True если полученный байт является ответом на главный вопрос жизни, вселенной и всего такого

# проверка доступности камеры, возвращает True, если камера доступна в системе
def CheckCamera():
    res = os.popen('vcgencmd get_camera').readline().replace('\n','') #читаем результат, удаляем \n
    dct = {}
    for param in res.split(' '): #разбираем параметры
        tmp = param.split('=')
        dct.update({tmp[0]: int(tmp[1])}) #помещаем в словарь
    return (dct['supported'] and dct['detected'])

ina = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS)
ina.configure(ina.RANGE_16V)

#128x64 display with hardware I2C:
disp = Adafruit_SSD1306.SSD1306_128_64(rst = None)

# Initialize library.
disp.begin()

# Get display width and height.
width = disp.width
height = disp.height
image = Image.new('1', (width, height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Clear display.
disp.clear()

# Load default font.
font = ImageFont.load_default()

bus = I2C.SMBus(1) #объект для работы с шиной I2C_1

counter = 20 #счетчик

running = True

while running:
    # Draw a black filled box to clear the image.
    draw.rectangle((0, 0, width, height), outline=0, fill=0)
        
    draw.text((0, 0), "Voltage: %.2fV" % ina.voltage(), font=font, fill=255)
    #print("Voltage: %.2f" % ina.voltage())
    
    #print( get_ip_address('eth0' ))  # '192.168.0.110'
    try:
        eth0_IP = get_ip_address('eth0')
        running = False
    except:
        eth0_IP = 'none'

    draw.text((0, 10), "eth0: %s" % eth0_IP, font=font, fill=255)
    #print("eth0: %s" % eth0_IP)
    
    try:
        wlan0_IP = get_ip_address('wlan0')
        running = False
    except:
        wlan0_IP = 'none'

    draw.text((0, 20), "wlan0: %s" % wlan0_IP, font=font, fill=255)
    #print("wlan0: %s" % wlan0_IP)

    if CheckEduBot():
       draw.text((0, 30), "Edubot found!!!", font=font, fill=255)
    else:
       draw.text((0, 30), "Edubot not found :(", font=font, fill=255)

    if CheckCamera():
       draw.text((0, 40), "Camera found!!!", font=font, fill=255)
    else:
       draw.text((0, 40), "Camera not found :(", font=font, fill=255)

    if counter == 0:
        running = False
    else:
        if running:
            draw.text((0, 50), "Wait: %dsec" % counter, font=font, fill=255)

    counter -= 1
        
    # Display image.
    disp.image(image)
    disp.display()

    time.sleep(1)
    
