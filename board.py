#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#подлючение модулей
from time import sleep
from xmlrpc.server import SimpleXMLRPCServer
import threading

import edubot

IP = '173.1.0.29'
PORT = 8000

servoPos = 62

#поток для контроля напряжения и тока
#и отображения на дисплее
class StateThread(threading.Thread):
    
    def __init__(self, robot):
        super(StateThread, self).__init__()
        self.daemon = True
        self._stopped = threading.Event() #событие для остановки потока
        self._robot = robot

    def run(self):
        image = Image.new('1', self._robot.displaySize) #создаем ч/б картинку для отрисовки на дисплее
        draw = ImageDraw.Draw(image) #создаем объект для рисования на картинке
        font = ImageFont.load_default() #создаем шрифт для отрисовки на дисплее

        print('State thread started')
        while not self._stopped.is_set():
            # Отрисовываем на картинке черный прямоугольник, тем самым её очищая
            draw.rectangle((0, 0, self._robot.displaySize[0], self._robot.displaySize[1]), outline=0, fill=0)
            
            #Отрисовываем строчки текста с текущими значениями напряжения, сылы тока и мощности
            draw.text((0, 0), "Edubot project", font=font, fill=255)
            draw.text((0, 10), "Voltage: %.2fV" % self._ina.voltage(), font=font, fill=255)
            draw.text((0, 20), "Current: %.2fmA" % self._ina.current(), font=font, fill=255)
            draw.text((0, 30), "Power: %.2f" % self._ina.power(), font=font, fill=255)

            # Отображаем картинку на дисплее
            self._robot.DrawImage(image)
            
            sleep(1)
            
        print('State thread stopped')

    def stop(self): #остановка потока
        self._stopped.set()
        self.join()

def Speak():
    # бибикнуть
    robot.Beep()
    return 0

def SetSpeed(leftSpeed, rightSpeed):
    robot.leftMotor.SetSpeed(leftSpeed)
    robot.rightMotor.SetSpeed(rightSpeed)
    return 0

def StopMotor():    
    robot.leftMotor.SetSpeed(0)
    robot.rightMotor.SetSpeed(0)
    return 0

def ServoUp():
    global servoPos
    servoPos -= 10
    if servoPos < 0:
        servoPos = 0
    robot.servo[0].SetPosition(servoPos)
    print ('ServoPos = %d' % servoPos)
    return 0

def ServoDown():
    global servoPos
    servoPos += 10
    if servoPos > 125:
        servoPos = 125
    robot.servo[0].SetPosition(servoPos)
    print ('ServoPos = %d' % servoPos)
    return 0

robot = edubot.EduBot(enableDisplay = True)
assert robot.Check(), 'EduBot not found!!!'
robot.Start() #обязательная процедура, запуск потока отправляющего на контроллер EduBot онлайн сообщений

print ('EduBot started!!!')

robot.ClearDisplay() #очистка дисплея

#создаем и запускаем поток отображающий данные телеметрии да дисплее робота
stateThread = StateThread(robot)
stateThread.start()

# Создаем сервер (IP адрес, порт, отключен лог)
server = SimpleXMLRPCServer((IP, PORT), logRequests = False)
print('Control XML-RPC server listening on %s:%d' % (IP, PORT))

# регистрируем функции
server.register_function(Speak)
server.register_function(SetSpeed)
server.register_function(StopMotor)
server.register_function(ServoUp)
server.register_function(ServoDown)

# запускаем сервер
try:
    server.serve_forever()
except KeyboardInterrupt:
    print('Ctrl+C pressed')
    
robot.servo[0].SetPosition(62)
robot.leftMotor.SetSpeed(0)
robot.rightMotor.SetSpeed(0)
stateThread.stop()

robot.ClearDisplay() #очистка дисплея

#останавливаем поток отправки онлайн сообщений в контроллер EduBot
robot.Release()
print('Stop program')
