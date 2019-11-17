#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

#библиотеки для работы с изображениями Python Image Library
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

#библиотека для работы с контроллером EduBot
import edubot

SPEED = 200 #скорость моторов
SERVO_MID_POS = 62 #среднее положение сервомотора

robot = edubot.EduBot(enableDisplay = True) #создаем обект для работы с EduBot, робот с OLED дисплеем

assert robot.Check(), 'EduBot not found!!!' #проверяем наличие платы Edubot

image = Image.new('1', robot.displaySize) #создаем ч/б картинку для отрисовки на дисплее
draw = ImageDraw.Draw(image) #создаем объект для рисования на картинке
font = ImageFont.load_default() #создаем шрифт для отрисовки текста на картинке

robot.ClearDisplay() #очистка дисплея

robot.Start() #обязательная процедура, запуск потока отправляющего на контроллер EduBot онлайн сообщений
print ('EduBot started!!!')

shiftServoPos = 20

robot.Beep() #бибикаем

try:
    while True:
        #крутим моторами
        robot.leftMotor.SetSpeed(SPEED)
        robot.rightMotor.SetSpeed(SPEED)
        time.sleep(1)
        #останавливаем моторы
        robot.leftMotor.SetSpeed(0)
        robot.rightMotor.SetSpeed(0)
        time.sleep(1)

        #наклоняем серво
        robot.servo[0].SetPosition(SERVO_MID_POS + shiftServoPos)
        shiftServoPos = -shiftServoPos #инвертируем положение

        voltage, current, power = robot.GetPowerData() #получаем данные: напряжение, сила тока, мощность
        
        # Отрисовываем на картинке черный прямоугольник, тем самым её очищая
        draw.rectangle((0, 0, robot.displaySize[0], robot.displaySize[1]), outline=0, fill=0)
        #Отрисовываем строчки текста с текущими значениями напряжения, сылы тока и мощности
        draw.text((0, 0), "EDUBOT PROJECT", font=font, fill=255)
        draw.text((0, 10), "Voltage: %.2f" % voltage, font=font, fill=255)
        draw.text((0, 20), "Current: %.2f" % current, font=font, fill=255)
        draw.text((0, 30), "Power: %.2f" % power, font=font, fill=255)

        robot.DrawDisplay(image) #отображаем сформированную картинку на дисплее

            
except KeyboardInterrupt:
    print('Ctrl+C pressed')

robot.Beep() #бибикаем

robot.ClearDisplay() #очистка дисплея

#останов двигателей
robot.leftMotor.SetSpeed(0)
robot.rightMotor.SetSpeed(0)

#все сервы в среднее положение
robot.servo[0].SetPosition(SERVO_MID_POS)
robot.servo[1].SetPosition(SERVO_MID_POS)
robot.servo[2].SetPosition(SERVO_MID_POS)
robot.servo[3].SetPosition(SERVO_MID_POS)

#останавливаем поток отправки онлайн сообщений в контроллер EduBot
robot.Release()
print('Stop EduBot')
