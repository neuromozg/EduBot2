import smbus as I2C
import threading
import time
import logging

#библиотеки для работы с i2c монитором питания INA219
#установка библиотеки sudo pip3 install pi-ina219
from ina219 import INA219
from ina219 import DeviceRangeError

#библиетека для работы с OLED дисплеем
#установка библиотеки sudo pip3 install Adafruit-SSD1306
import Adafruit_SSD1306

I2C_1 = 1 #номер шины I2C

#I2C Address of device
EDUBOT_ADDRESS = 0x27

REG_WHY_IAM     = 0x00
REG_ONLINE      = 0x01
REG_SERVO_0     = 0x02
REG_SERVO_1     = 0x03
REG_MOTOR_MODE  = 0x04
REG_KP          = 0x05
REG_KI          = 0x06
REG_KD          = 0x07
REG_INT_SUMM    = 0x08
REG_PID_PERIOD  = 0x09
REG_PARROT_0    = 0x0A
REG_PARROT_1    = 0x0B
REG_DIR_0       = 0x0C
REG_PWM_0       = 0x0D
REG_DIR_1       = 0x0E
REG_PWM_1	= 0x0F
REG_RESET_ALL_MOTOR = 0x10
REG_BEEP	= 0x11
REG_BUTTON      = 0x12

MOTOR_MODE_PWM = 0
MOTOR_MODE_PID = 1

SHUNT_OHMS = 0.01 #значение сопротивления шунта на плате EduBot
MAX_EXPECTED_AMPS = 2.0

I2C_LOG_FORMAT = 'Error %d, %s accessing 0x%02X: Check your I2C address'

class _Motor():
    def __init__(self, bus, lock, regDir, regPWM, regParrot):
        self._bus = bus #объект для работы с шиной I2C
        self._busLock = lock #блокировка для раздельного доступа к I2C
        self._regDir = regDir
        self._regPWM = regPWM
        self._regParrot = regParrot
        
        
    def SetSpeed(self, speed):
        #нормализуем скорость
        if speed > 255:
            speed = 255
        elif speed < -255:
            speed = -255
            
        self._busLock.acquire()
        try:   
            try:
                #задаем направление
                direction = int(speed > 0)
                self._bus.write_byte_data(EDUBOT_ADDRESS, self._regDir, direction)
                time.sleep(0.001)
                #задаем ШИМ
                self._bus.write_byte_data(EDUBOT_ADDRESS, self._regPWM, abs(speed))
                time.sleep(0.001)
            except IOError as err:
                logging.exception(I2C_LOG_FORMAT, err.errno, err.strerror, EDUBOT_ADDRESS)
        finally:
            self._busLock.release()
    
    def SetParrot(self, parrot):
        #нормализуем скорость
        if parrot > 100:
            speed = 100
        elif speed < -100:
            speed = -100
            
        self._busLock.acquire()
        try:   
            try:
                #задаем направление
                direction = int(speed > 0)
                self._bus.write_byte_data(EDUBOT_ADDRESS, self._regDir, direction)
                time.sleep(0.001)
                #задаем ШИМ
                self._bus.write_byte_data(EDUBOT_ADDRESS, self._regParrot, abs(parrot))
                time.sleep(0.001)
            except IOError as err:
                logging.exception(I2C_LOG_FORMAT, err.errno, err.strerror, EDUBOT_ADDRESS)
        finally:
            self._busLock.release()



class _Servo():
    def __init__(self, bus, lock, numServo):
        self._bus = bus #объект для работы с шиной I2C
        self._busLock = lock #блокировка для раздельного доступа к I2C
        self._numServo = numServo

    def SetPosition(self, pos):
        #нормализуем позицию
        if pos < 0:
            pos = 0
        elif pos > 250:
            pos = 250
            
        self._busLock.acquire()
        try:
            try:
                #задаем позицию
                self._bus.write_byte_data(EDUBOT_ADDRESS, REG_SERVO_0 + self._numServo, pos)
                time.sleep(0.001)
            except IOError as err:
                logging.exception(I2C_LOG_FORMAT, err.errno, err.strerror, EDUBOT_ADDRESS)
        finally:
            self._busLock.release()

class _OnLiner(threading.Thread):
    def __init__(self, bus, lock):
        super(_OnLiner, self).__init__()
        self._bus = bus
        self._busLock = lock #блокировка для раздельного доступа к I2C
        self.daemon = True
        self._stopped = threading.Event() #событие для остановки потока

    def run(self):
        #print('OnLiner thread started')
        logging.info('OnLiner thread started')
        while not self._stopped.is_set():
            self._busLock.acquire()
            try:
                try:
                    self._bus.write_byte_data(EDUBOT_ADDRESS, REG_ONLINE, 1)
                    time.sleep(0.001)
                except IOError as err:
                    logging.exception(I2C_LOG_FORMAT, err.errno, err.strerror, EDUBOT_ADDRESS)
            finally:
                self._busLock.release()
            time.sleep(1)
            
        logging.info('OnLiner thread stopped')
        #print('OnLiner thread stopped')

    def stop(self): #остановка потока
        self._stopped.set()
        self.join()
          
class EduBot():
    def __init__(self, enableDisplay = False):
        self._bus = I2C.SMBus(I2C_1) #объект для работы с шиной I2C
        self.busLock = threading.Lock() #блокировка для раздельного доступа к I2C
        self.leftMotor = _Motor(self._bus, self.busLock, REG_DIR_0, REG_PWM_0, REG_PARROT_0)
        self.rightMotor = _Motor(self._bus, self.busLock, REG_DIR_1, REG_PWM_1, REG_PARROT_1)
        self._servo0 = _Servo(self._bus, self.busLock, 0)
        self._servo1 = _Servo(self._bus, self.busLock, 1)
        self.servo = (self._servo0, self._servo1)
        self._onLiner = _OnLiner(self._bus, self.busLock)

        self.busLock.acquire()
        try:
            self._ina = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS) #создаем обект для работы с INA219
            self._ina.configure(INA219.RANGE_16V) #конфигурируем INA219
        finally:
            self.busLock.release()

        self._display = None
        self.displaySize = (0, 0)
        
        self.enableDisplay = enableDisplay        
        if enableDisplay:
            self._display = Adafruit_SSD1306.SSD1306_128_64(rst = None) #создаем обект для работы c OLED дисплеем 128х64

            self.busLock.acquire()
            try:
                self._display.begin() #инициализируем дисплей
            finally:
                self.busLock.release()
            self.displaySize = (self._display.width, self._display.height);
            
    def SetMotorMode(self, mode):
        self._busLock.acquire()
        try:
            try:
                #задаем режим работы моторов
                self._bus.write_byte_data(EDUBOT_ADDRESS, REG_MOTOR_MODE, mode)
                time.sleep(0.001)
            except IOError as err:
                logging.exception(I2C_LOG_FORMAT, err.errno, err.strerror, EDUBOT_ADDRESS)
        finally:
            self._busLock.release()       
            
    def ClearDisplay(self):
        if self.enableDisplay:
            # Очистка дисплея
            self.busLock.acquire()
            try:
                self._display.clear() #очищаем дисплей
                self._display.display() #обновляем дисплей
            finally:
                self.busLock.release()            

    def DrawDisplay(self, image):
        if self.enableDisplay:
            # Очистка дисплея
            self.busLock.acquire()
            try:
                self._display.image(image) # Копируем картинку на дисплей
                self._display.display() #обновляем дисплей
            finally:
                self.busLock.release() 
            
    def Check(self):
        self.busLock.acquire()
        try:
            try:
                res = self._bus.read_byte_data(EDUBOT_ADDRESS, REG_WHY_IAM)
                time.sleep(0.001)
            except IOError as err:
                logging.exception(I2C_LOG_FORMAT, err.errno, err.strerror, EDUBOT_ADDRESS)
        finally:
            self.busLock.release()
        return (res == 0X2A) #возвращает True если полученный байт является ответом на главный вопрос жизни, вселенной и всего такого

    def Start(self):
        self._onLiner.start()

    def Release(self):
        self._onLiner.stop()

    def Beep(self):
        self.busLock.acquire()
        try:
            try:
                self._bus.write_byte_data(EDUBOT_ADDRESS, REG_BEEP, 3)
                time.sleep(0.001)
            except IOError as err:
                logging.exception(I2C_LOG_FORMAT, err.errno, err.strerror, EDUBOT_ADDRESS)
        finally:
            self.busLock.release()

    #функция возвращает значение напряжения, силы тока, мощности в текущий момент времени
    def GetPowerData(self):
        voltage = 0
        current = 0
        power = 0
        
        self.busLock.acquire()
        try:
            voltage = self._ina.voltage()
            current = self._ina.current()
            power = self._ina.power()
        finally:
            self.busLock.release()
        
        return voltage, current, power


        

    
