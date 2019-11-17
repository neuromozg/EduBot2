/*
 * main.c
 *
 *  Created on: 29.10.2019
 *      Author: max
 */


// Atmega328P
// 16MHz

#include <avr/io.h>
//#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include <compat/twi.h>

//RTOS librares
#include "include/rtos.h"

//#define TEST
#define RTOS_FREQ 1000U // частота работы таймера RTOS
#define WAIT_ONLINE_SEC 3 		//количество секунд за время которых должен прибыть сигнал онлайн

#define LED_INF_BIT PB0 //выход информационного светодиода
#define LED_INF_DDR DDRB
#define LED_INF_PORT PORTB

//Порты мотора A
//ШИМ
#define MOTOR_A_PWM_BIT PD6
#define MOTOR_A_PWM_DDR DDRD
#define MOTOR_A_PWM_PORT PORTD
#define MOTOR_A_PWM_PIN PIND

//Направление вращения
#define MOTOR_A_CW_BIT PC2
#define MOTOR_A_CW_DDR DDRC
#define MOTOR_A_CW_PORT PORTC
#define MOTOR_A_CW_PIN PINC

//Вход датчика хола
#define MOTOR_A_FG_BIT PD2
#define MOTOR_A_FG_DDR DDRD
#define MOTOR_A_FG_PORT PORTD
#define MOTOR_A_FG_PIN PIND

#define MOTOR_A_PWM_TIMER_REG OCR0A

//Порты мотора B
//ШИМ
#define MOTOR_B_PWM_BIT PD5
#define MOTOR_B_PWM_DDR DDRD
#define MOTOR_B_PWM_PORT PORTD
#define MOTOR_B_PWM_PIN PIND

//Направление вращения
#define MOTOR_B_CW_BIT PD4
#define MOTOR_B_CW_DDR DDRD
#define MOTOR_B_CW_PORT PORTD
#define MOTOR_B_CW_PIN PIND

//Вход датчика хола
#define MOTOR_B_FG_BIT PD3
#define MOTOR_B_FG_DDR DDRD
#define MOTOR_B_FG_PORT PORTD
#define MOTOR_B_FG_PIN PIND

#define MOTOR_B_PWM_TIMER_REG OCR0B

//Серво0
#define SERVO0_BIT PB1
#define SERVO0_DDR DDRB
#define SERVO0_PORT PORTB
#define SERVO0_PIN PINB

#define SERVO0_PWM_TIMER_REG OCR1A

//Серво1
#define SERVO1_BIT PB2
#define SERVO1_DDR DDRB
#define SERVO1_PORT PORTB
#define SERVO1_PIN PINB

#define SERVO1_PWM_TIMER_REG OCR1B

//Бипер
#define BUZZER_BIT PC3
#define BUZZER_DDR DDRC
#define BUZZER_PORT PORTC
#define BUZZER_PIN PINC

//Кнопка
#define BUTTON_BIT PC0
#define BUTTON_DDR DDRC
#define BUTTON_PORT PORTC
#define BUTTON_PIN PINC

//режимы работы моторов
#define MOTOR_MODE_PWM 0 //управляем двигателями напрямую через задание PWM
#define MOTOR_MODE_PID 1 //управляем двигателями через ПИД регулятор

#define SERVO_BASE_TIMER 250 //значение тиков таймера минимального сигнала 1000мс
#define SERVO_MIN 0
#define SERVO_MED 128
#define SERVO_MAX 250

//i2c
#define I2C_SLAVE_ADDR      0x27

#define REG_WHY_IAM			0x00
#define REG_ONLINE			0x01
#define REG_SERVO0			0x02
#define REG_SERVO1			0x03
#define REG_MOTOR_MODE		0x04
#define REG_KP				0x05
#define REG_KI				0x06
#define REG_KD				0x07
#define REG_INT_SUMM		0x08
#define REG_PID_PERIOD		0x09
#define REG_PARROT_A		0x0A
#define REG_PARROT_B		0x0B
#define REG_DIR_A			0x0C
#define REG_PWM_A			0x0D
#define REG_DIR_B			0x0E
#define REG_PWM_B			0x0F
#define REG_RESET_ALL_MOTOR	0x10
#define REG_BEEP			0x11
#define REG_BUTTON			0x12

//------Параметры системы сохраняемые в EEPROM----------------------------
typedef struct {
	uint8_t workMode; //режим работы контроллера
	float kP;  							// Пропорциональный коэффициент
	int8_t kPint;
	float kI;  							// Интегральный коэффициент
	int8_t kIint;
	float kD;  							// Дифференциальный коэффициент
	int8_t kDint;
	int16_t limitSum;					// Ограничение интегральной суммы
	uint16_t timePID;					// Время обсчета ПИД-регулятора
} WorkParams;

typedef void (*pSetPIDMotor)(int16_t);

typedef struct {
	char name; //имя канала
	volatile uint8_t* timerPWMReg;		//канал таймера
	volatile uint8_t* portCw; //порт выхода направления вращения
	volatile uint8_t* pinCw; //пин выхода направления вращения
	uint8_t bitCwMask; //маска бит выхода направления вращения
	volatile uint8_t* pinFg; //пин входа датчика холла
	uint8_t bitFgMask; //маска бит входа датчика холла
	uint8_t dir; //направление вращения при задании через i2c
	int32_t odomCount; //счетчик одометрии
	int32_t odomCountOld;
	int16_t setParrot;	//заданные попугаи
	int16_t integralSum; //интегральная сумма
	int16_t oldErrorParrot;	//предыдущее значение попугаев
	int16_t currentParrot; 	//текущие попугаи
	float pParrot; //пропорциональная составляющая
	float iParrot; //интегральная составляющая
	float dParrot; //дифференциальная составляющая
	int16_t resPID; //результат ПИД
	pSetPIDMotor SetPIDMotor;
} MotorData;

WorkParams workParams;

MotorData motorA;
MotorData motorB;

uint8_t regAddr; // Store the Requested Register Address
uint8_t regData; // Store the Register Address Data

uint8_t tagOnline; //признак онлайна
uint8_t countOffline = 0; //счетчик пропусков
uint16_t pauseIndicationOnline = 100; //задежка в мс для световой индикации при онлайне
uint16_t pauseIndicationOffline = 1000; //задежка в мс для световой индикации при оффлайне

uint8_t tagBeep; //признак, что сейчас пищим

#if defined(TEST)
int16_t testPWM;
int16_t testParrot;
uint8_t testState;
#endif

void StopMotor(MotorData *data);
void OdometrProcess(MotorData *data);
void I2CSlaveAction(uint8_t rwStatus);
void ResetMotorData(MotorData *data);
void SetMotorPWM(MotorData *data, uint8_t pwm);
void SetMotorDir(MotorData *data, uint8_t dir);
void SetMotorDirPWM(MotorData *data, int16_t pwm);
void ServoSetPos(volatile uint8_t* timerReg, uint8_t pos);

//задачи RTOS
void ApplyWorkMode(void *data);
void CalcPID(MotorData *data);
void UpdatePID(void *data);
void CalcParrot(MotorData *data);
void UpdateParrot(void *data);
void OffLineEvent(void *data);
void OnLineEvent(void *data);
void AcceptedOnlineLabel(void *data);
void StartBeep(void *data);
void StopBeep(void *data);

ISR(TWI_vect)
{
    static uint8_t i2c_state;

    // Disable Global Interrupt
    cli();

    // Get TWI Status Register, mask the prescaler bits (TWPS1,TWPS0)
    uint8_t twi_status = TW_STATUS;

    switch(twi_status) {
        case TW_SR_SLA_ACK: // 0x60: SLA+W received, ACK returned
            i2c_state=0;    // Start I2C State for Register Address required
            break;

        case TW_SR_DATA_ACK:    // 0x80: data received, ACK returned
            if (i2c_state == 0) {
                regAddr = TWDR; // прочитал номер регистра
                i2c_state = 1;
            } else {
                regData = TWDR; // прочитал данные
                i2c_state = 2;
            }
            break;

        case TW_SR_STOP:    // 0xA0: stop or repeated start condition received while selected
            if (i2c_state == 2) {
            	I2CSlaveAction(1);    // Call Write I2C Action (rw_status = 1)
                i2c_state = 0;      // Reset I2C State
            }
            break;

        case TW_ST_SLA_ACK: // 0xA8: SLA+R received, ACK returned
        case TW_ST_DATA_ACK:    // 0xB8: data transmitted, ACK received
            if (i2c_state == 1) {
            	I2CSlaveAction(0);    // Call Read I2C Action (rw_status = 0)
                TWDR = regData;     // Store data in TWDR register
                i2c_state = 0;      // Reset I2C State
            }
            break;

        case TW_ST_DATA_NACK:   // 0xC0: data transmitted, NACK received
        case TW_ST_LAST_DATA:   // 0xC8: last data byte transmitted, ACK received
        case TW_BUS_ERROR:  // 0x00: illegal start or stop condition
        default:
            i2c_state = 0;  // Back to the Begining State
    }

    // Clear TWINT Flag
    TWCR |= (1<<TWINT);
    // Enable Global Interrupt
    sei();
}

//прерывание по изменению состояния входа от датчика хола A
ISR(INT0_vect)
{
	OdometrProcess(&motorA);
}

//прерывание по изменению состояния входа от датчика хола B
ISR(INT1_vect)
{
	OdometrProcess(&motorB);
}

void I2CSlaveAction(uint8_t rwStatus)
{
	switch(regAddr) {

	// PORT
	case REG_WHY_IAM:
		if (rwStatus == 0)
			// read
			regData = 0x2A; //Ответ на главный вопрос жизни, вселенной и всего такого
		break;

	case REG_ONLINE:
		if (rwStatus)
		{
			SetTask(AcceptedOnlineLabel);
		}
		break;
	case REG_SERVO0:
		if (rwStatus)
		{
			if (tagOnline)
				if (regData > SERVO_MAX)
					regData = SERVO_MAX;

				SERVO0_PWM_TIMER_REG = SERVO_BASE_TIMER + regData;
		}
		else
			//regData = servo[0].position;
			regData = 0;
		break;
	case REG_SERVO1:
		if (rwStatus)
		{
			if (tagOnline)
				if (regData > SERVO_MAX)
					regData = SERVO_MAX;
				SERVO1_PWM_TIMER_REG = SERVO_BASE_TIMER + regData;
		}
		else
			//regData = servo[1].position;
			regData = 0;
		break;

	case REG_MOTOR_MODE:
		if (rwStatus == 0)
			// read
			regData = workParams.workMode;
		else
		{
			workParams.workMode = regData;
			SetTask(ApplyWorkMode);
		}
		break;

	case REG_KP:
		if (rwStatus == 0)
			// read
			regData = workParams.kPint;
		else
		{
			workParams.kPint = regData;
			workParams.kP = regData/10;
		}
		break;

	case REG_KI:
		if (rwStatus == 0)
			// read
			regData = workParams.kIint;
		else
		{
			workParams.kIint = regData;
			workParams.kI = regData/10;
		}
		break;

	case REG_KD:
		if (rwStatus == 0)
			// read
			regData = workParams.kDint;
		else
		{
			workParams.kDint = regData;
			workParams.kD = regData/10;
		}
		break;

	case REG_INT_SUMM:
		if (rwStatus == 0)
			// read
			regData = workParams.limitSum >> 1;
		else
			workParams.limitSum = regData << 1;
		break;

	case REG_PID_PERIOD:
		if (rwStatus == 0)
			// read
			regData = workParams.timePID >> 1;
		else
			workParams.timePID = regData << 1;
		break;

	case REG_PARROT_A:
		if (rwStatus == 0)
			// read
			regData = motorA.setParrot;
		else
			if (workParams.workMode == MOTOR_MODE_PID)
			{
				//задаем попугаи в зависимости от направления вращения
				if (motorA.dir)
					motorA.setParrot = regData;
				else
					motorA.setParrot = (-1)*regData;
			}
		break;

	case REG_PARROT_B:
		if (rwStatus == 0)
			// read
			regData = motorB.setParrot;
		else
			if (workParams.workMode == MOTOR_MODE_PID)
			{
				//задаем попугаи в зависимости от направления вращения
				if (motorB.dir)
					motorB.setParrot = regData;
				else
					motorB.setParrot = (-1)*regData;
			}
		break;

	case REG_DIR_A:
		if (rwStatus == 0)
			// read
			regData = (*motorA.pinCw) & motorA.bitCwMask;
		else
		{
			motorA.dir = regData;
		}
		break;

	case REG_PWM_A:
		if (rwStatus == 0)
			// read
			regData = *motorA.timerPWMReg;
		else
		{
			if (workParams.workMode == MOTOR_MODE_PWM)
			{
				//задаем направление вращения
				if (motorA.dir)
					(*motorA.portCw) |= motorA.bitCwMask;
				else
					(*motorA.portCw) &= ~motorA.bitCwMask;

				//задаем скорость вращения
				*motorA.timerPWMReg = regData;
			}
		}
		break;

	case REG_DIR_B:
		if (rwStatus == 0)
			// read
			regData = (*motorB.pinCw) & motorB.bitCwMask;
		else
		{
			motorB.dir = regData;
		}
		break;

	case REG_PWM_B:
		if (rwStatus == 0)
			// read
			regData = *motorB.timerPWMReg;
		else
		{
			if (workParams.workMode == MOTOR_MODE_PWM)
			{
				//задаем направление вращения
				if (motorB.dir)
					(*motorB.portCw) |= motorB.bitCwMask;
				else
					(*motorB.portCw) &= ~motorB.bitCwMask;

				//задаем скорость вращения
				*motorB.timerPWMReg = regData;
			}
		}
		break;

	case REG_RESET_ALL_MOTOR:
		if (rwStatus)
		{
			SetTaskParam((TPTR)ResetMotorData, &motorA);
			SetTaskParam((TPTR)ResetMotorData, &motorB);
		}
		break;
	case REG_BEEP:
		if (rwStatus)
		{
			SetTask(StartBeep);
		}
		break;
	default:
		regData = 0x00;
	}
}

void ServoSetPos(volatile uint8_t* timerReg, uint8_t pos) //задать положение сервопривода
{
	if (pos > SERVO_MAX)
		pos = SERVO_MAX;

	(*timerReg) = pos;
}

void StopMotor(MotorData *data)
{
	SetMotorPWM(data, 0);
}

void ResetMotorData(MotorData *data)
{
	data->odomCount = 0; //счетчик одометрии
	data->odomCountOld = 0;
	data->setParrot = 0;	//заданные попугаи
	data->integralSum = 0; //интегральная сумма
	data->oldErrorParrot = 0;	//предыдущее значение попугаев
	data->currentParrot = 0; 	//текущие попугаи
	data->pParrot = 0; //пропорциональная составляющая
	data->iParrot = 0; //интегральная составляющая
	data->dParrot = 0; //дифференциальная составляющая
	data->resPID = 0; //результат ПИД
	data->dir = 0; //направление вращения
}

void OdometrProcess(MotorData *data)
{
	if (*data->pinCw & data->bitCwMask) //в зависимости от направления вращения
		data->odomCount++; //увеличиваем счетчик одометра
	else
		data->odomCount--; //уменьшаем счетчик одометра
}

void CalcPID(MotorData *data)
{
	int16_t errorParrot; //ошибка попугаев
	int32_t odomCount;
	int16_t setParrot;
	//uint16_t stepIntegralDecrement;

	cli(); //получаем необходимые данные, защитившись от прерываний
	odomCount = data->odomCount; //текущие показания одометра (изменяется по таймеру)
	setParrot = data->setParrot; //уставка, заданное значение попугаев (задается по i2c)
	sei();

	data->currentParrot = odomCount - data->odomCountOld;		//вычисление текущего попугая (тахометр)
	data->odomCountOld = odomCount;

	errorParrot = setParrot - data->currentParrot; //вычисление ошибки

	data->integralSum += errorParrot;          // вычисление интеграла (интегральной суммы ошибок)

	/*
	//вычисление интеграла через циклический буффер
	Parms->integralBuff[Parms->integralCount] = errorParrot; //добавляем ошибку в массив
	Parms->integralCount++;
	if (Parms->integralCount > (IntegralBuffelLength-1)) //проверка на переполнение счетчика
		Parms->integralCount = 0;

	//вычисление интеграла
	Parms->integralSum = 0;
	for (uint8_t i = 0; i<IntegralBuffelLength; i++)
		Parms->integralSum += Parms->integralBuff[i]; //суммируем данные кольцевого буффера
	*/

	if (data->integralSum > workParams.limitSum) //нормализация интеграла в пределах заданных лимитов
		data->integralSum = workParams.limitSum;
	else
	{
		if (data->integralSum < -workParams.limitSum)
			data->integralSum = -workParams.limitSum;
	}

	//Возможно костыль, затухание интегральной суммы до 0
	//Сделано, в моменты когда задан 0, мотор остановился т.е. тоже 0, а интегральная сумма остатлась и вносит свое значение в результат ПИД
	if ((setParrot == 0) && (data->currentParrot == 0) && (data->integralSum != 0))
	{
		if (data->integralSum > 0)
			data->integralSum --;
		else
			if (data->integralSum < 0)
				data->integralSum ++;
	}

	//уменьшение интеграла при заданных нулевых попугаях,
	//сделано для того, чтобы постепенно обнулить интегральную составляющую, когда двигатель остановлен
	//и ПИД регулятор уже не работает, но инт. сумма влияет на ШИМ
//	if ((data->setParrot == 0) && (data->currentParrot == 0) && (data->integralSum != 0))
//	{
//		stepIntegralDecrement = workParams.timePID >> 2; //шаг уменьшения интеграла, время частоты ПИД делим на 4, пока реализовано так
//		if (stepIntegralDecrement == 0)
//			stepIntegralDecrement = 1;
//
//		if (data->integralSum > 0)
//		{
//			if (data->integralSum > stepIntegralDecrement) //уменьшение положительной суммы
//				data->integralSum -= stepIntegralDecrement;
//			else
//				data->integralSum = 0;
//		}
//		else
//		{
//			if (data->integralSum < -stepIntegralDecrement) //уменьшение отрицательной суммы
//				data->integralSum += stepIntegralDecrement;
//			else
//				data->integralSum = 0;
//		}
//	}

	data->pParrot = workParams.kP * errorParrot;    // пропорциональная составляющая
	data->iParrot = workParams.kI * data->integralSum;    // интегральная составляющая
	data->dParrot = workParams.kD * (errorParrot - data->oldErrorParrot); //дифференциальная составляющая
	data->oldErrorParrot = errorParrot; //сохранил ошибку
	data->resPID = data->pParrot + data->iParrot + data->dParrot; //вычисление результата ПИД

	SetMotorDirPWM(data, data->resPID);

//	printf_P(PSTR("odo:%d set:%d curr:%d P:%.2f I:%.2f D:%.2f res: %d\n"), (int)odomCount, (int)setParrot, (int)motorA.currentParrot,
//			(float)motorA.pParrot, (float)motorA.iParrot, (float)motorA.dParrot, (int)motorA.resPID); //одометры

}

void SetMotorDir(MotorData *data, uint8_t dir)
{
	//задаем направление вращения
	if (dir)
	{
		(*data->portCw) |= data->bitCwMask; //задаем напраление вращения
	}
	else
	{
		(*data->portCw) &= ~data->bitCwMask; //задаем напраление вращения
	}
}

void SetMotorPWM(MotorData *data, uint8_t pwm)
{
	pwm = ~pwm; //Инвертируем PWM для моторов 25D

	*data->timerPWMReg = pwm; //задаем значение ШИМ в регистр таймера
}

void SetMotorDirPWM(MotorData *data, int16_t pwm)
{
	uint16_t timerPWM = abs(pwm);

	if (timerPWM > 255)
		timerPWM = 255;

	SetMotorPWM(data, timerPWM);
	SetMotorDir(data, (pwm > 0));
}

void UpdatePID(void *data)
{
	if (workParams.workMode == MOTOR_MODE_PID)
	{
		SetTaskParam((TPTR)CalcPID, &motorA); //ставим задачу вычисления ПИД 1-го мотора
		SetTaskParam((TPTR)CalcPID, &motorB);

		SetTimerTask(UpdatePID, workParams.timePID); //повторный вызов задачи
	}
}

void CalcParrot(MotorData *data)
{
	int32_t odomCount;


	cli(); //получаем необходимые данные, защитившись от прерываний
	odomCount = data->odomCount; //текущие показания одометра (изменяется по таймеру)
	sei();

	data->currentParrot = odomCount - data->odomCountOld;		//вычисление текущего попугая (тахометр)
	data->odomCountOld = odomCount;


	//data->MotorGo(Parms->resPWM); //задаем ШИМ и направления вращения на драйвер мотора
	//SetTaskParam(&Parms->MotorGo, Parms->resPWM); //выполняем процедуру установки параметров ШИМ и направления вращения на драйвер мотора
}

void UpdateParrot(void *data)
{
	if (workParams.workMode == MOTOR_MODE_PWM)
	{
		SetTaskParam((TPTR)CalcParrot, &motorA); //ставим задачу вычисления оборотов
		SetTaskParam((TPTR)CalcParrot, &motorB);

		SetTimerTask(UpdateParrot, workParams.timePID); //повторный вызов задачи
	}
}

void ApplyWorkMode(void *data)
{
	switch(workParams.workMode)
	{
	case MOTOR_MODE_PWM:
		SetTask((TPTR)UpdateParrot);
		break;
	case MOTOR_MODE_PID:
		SetTask((TPTR)UpdatePID);
		break;
	}
}

void IndicationOnLine(void *data)
{
	LED_INF_PORT ^= 1<<LED_INF_BIT; //инвертируем светодиод

	if (tagOnline)
		SetTimerTask(IndicationOnLine, pauseIndicationOnline); //если онлайн то мигаем быстро
	else
		SetTimerTask(IndicationOnLine, pauseIndicationOffline); //если оффлайн мигаем раз в секунду
}

void ControlOnline(void *data)
{
	if ((countOffline < WAIT_ONLINE_SEC) && tagOnline)
		countOffline++;
	else
	{
		if (tagOnline)
		{
			tagOnline = FALSE;
			SetTask(OffLineEvent); //вызываем обработчик онлайн события
		}
	}

	SetTimerTask(ControlOnline, pauseIndicationOffline); //перезапуск задачи через 1 сек
}

void AcceptedOnlineLabel(void *data)
{
	countOffline = 0; //сброс счетчика отсутствия связи
	if (tagOnline == FALSE)
	{
		tagOnline = TRUE; //онлайн
		SetTask(OnLineEvent); //вызываем обработчик онлайн события
	}
}

void StartControlOnline(const uint16_t freqRTOS)
{
	//вычисляем частоту миганий светодиода в зависимости от частоты RTOS
	pauseIndicationOnline = freqRTOS >> 3; //частота индикации при онлайне равна 1/8 сек
	pauseIndicationOffline = freqRTOS; //частота индикации при оффлайне равна 1 сек

	SetTask(ControlOnline); //запуск задачи контроля соединения с ПО верхнего уровня (Online)
	SetTask(IndicationOnLine); //запуск задачи индикации соединения (мигаем светодиодом)
}

void OffLineEvent(void *data)
{
	//останов моторов
	StopMotor(&motorA);
	StopMotor(&motorB);

	//серво в среднее положение
	SERVO0_PWM_TIMER_REG = SERVO_BASE_TIMER + SERVO_MED;
	SERVO1_PWM_TIMER_REG = SERVO_BASE_TIMER + SERVO_MED;
}

void OnLineEvent(void *data)
{
	SetTask(StartBeep);
}

void StartBeep(void *data)
{
	if (tagBeep == FALSE)
	{
		tagBeep = TRUE;
		BUZZER_PORT |= 1 << BUZZER_BIT;
		SetTimerTask(StopBeep, 500);
	}

}

void StopBeep(void *data)
{
	BUZZER_PORT &= ~(1 << BUZZER_BIT);
	tagBeep = FALSE;
}

#if defined(TEST)
void TestMotor(void *data)
{
	testState = ~testState;
	if (testState)
	{
		SetTask(StartBeep);

		if (workParams.workMode == MOTOR_MODE_PWM)
		{
			testPWM = ~testPWM;
			SetMotorDirPWM(&motorA, testPWM);
			SetMotorDirPWM(&motorB, ~testPWM);
		}
		else
		{
			testParrot = -testParrot;
			motorA.setParrot = testParrot;
			motorB.setParrot = -testParrot;
		}
	}
	else
	{
		if (workParams.workMode == MOTOR_MODE_PWM)
		{
			StopMotor(&motorA);
			StopMotor(&motorB);
		}
		else
		{
			motorA.setParrot = 0;
			motorB.setParrot = 0;
		}
	}
	SetTimerTask(TestMotor, 3000);
}

void TestServo(void *data)
{
	if (SERVO0_PWM_TIMER_REG == SERVO_BASE_TIMER)
	{
		SERVO0_PWM_TIMER_REG = SERVO_BASE_TIMER + SERVO_MAX;
		SERVO1_PWM_TIMER_REG = SERVO_BASE_TIMER + SERVO_MAX;
	}
	else
	{
		SERVO0_PWM_TIMER_REG = SERVO_BASE_TIMER;
		SERVO1_PWM_TIMER_REG = SERVO_BASE_TIMER;
	}
	SetTimerTask(TestServo, 2000);
}
#endif

int main(void)
{
	wdt_enable(WDTO_500MS); //активирую сторожевой таймер

	//инициализируем таймер 0 для генерации ШИМ управления оборотами двигателя
	TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00); //подключаем ноги к выходам, режим FastPWM
	TCCR0B = (3 << CS00); //делитель 64, шим 976гц

	//инициализируем таймер 1 для генерации ШИМ управления сервомоторами
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11); //подключаем ноги к выходам, режим FastPWM сброс таймера по регистру OCR1 режим 1
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (3 << CS00); //делитель 64
	ICR1 = 4999; //шим 20гц

	SERVO0_PWM_TIMER_REG = SERVO_BASE_TIMER + SERVO_MED; //сервы в среднее положение
	SERVO1_PWM_TIMER_REG = SERVO_BASE_TIMER + SERVO_MED;

	//подключение входов и разрешение прерываний для входов датчика холла
	EICRA = (3 << ISC10) | (3 << ISC00); //ловим восходящий фронт
	EIMSK = (1 << INT1) | (1 << INT0); //разрешаем прерывания для входов INT1, INT0

	//конфигурируем порты IO
	//светодиоды
	LED_INF_DDR |= (1 << LED_INF_BIT);

	//выходы ШИМ и напраления вращения мотора А
	MOTOR_A_PWM_DDR |= (1 << MOTOR_A_PWM_BIT);
	MOTOR_A_CW_DDR |= (1 << MOTOR_A_CW_BIT);

	//выходы ШИМ и напраления вращения мотора B
	MOTOR_B_PWM_DDR |= (1 << MOTOR_B_PWM_BIT);
	MOTOR_B_CW_DDR |= (1 << MOTOR_B_CW_BIT);

	//подтягивающие резисторы для входа татчика хола (костыль, в следующих платах поправим)
	//MOTOR_A_FG_PORT |= (1 << MOTOR_A_FG_BIT);
	//MOTOR_B_FG_PORT |= (1 << MOTOR_B_FG_BIT);

	//звук
	BUZZER_DDR |= (1 << BUZZER_BIT);

	//выходы сервомоторов
	SERVO0_DDR |= (1 << SERVO0_BIT);
	SERVO1_DDR |= (1 << SERVO1_BIT);

	OCR0A = 0xFF; //на всякий случай шим на полную, моторы стоп
	OCR0B = 0xFF;

	//режим работы
	workParams.workMode = MOTOR_MODE_PWM;

	//прараметры работы
	workParams.kP = 0.5;  							// Пропорциональный коэффициент
	workParams.kPint = 5;
	workParams.kI = 0.5;  							// Интегральный коэффициент
	workParams.kIint = 5;
	workParams.kD = 0;  							// Дифференциальный коэффициент
	workParams.kDint = 0;
	workParams.limitSum = 500;					// Ограничение интегральной суммы
	workParams.timePID = 100;

	tagBeep = FALSE;
	tagOnline = FALSE;

	//Заполняем структуры
	motorA.name = 'A';
	motorA.portCw = &MOTOR_A_CW_PORT;
	motorA.pinCw = &MOTOR_A_CW_PIN;
	motorA.bitCwMask = (1 << MOTOR_A_CW_BIT);
	motorA.timerPWMReg = &MOTOR_A_PWM_TIMER_REG;
	motorA.pinFg = &MOTOR_A_FG_PIN;
	motorA.bitFgMask = (1 << MOTOR_A_FG_BIT);
	ResetMotorData(&motorA);

	motorB.name = 'B';
	motorB.portCw = &MOTOR_B_CW_PORT;
	motorB.pinCw = &MOTOR_B_CW_PIN;
	motorB.bitCwMask = (1 << MOTOR_B_CW_BIT);
	motorB.timerPWMReg = &MOTOR_B_PWM_TIMER_REG;
	motorB.pinFg = &MOTOR_B_FG_PIN;
	motorB.bitFgMask = (1 << MOTOR_B_FG_BIT);
	ResetMotorData(&motorB);

	StopMotor(&motorA);
	StopMotor(&motorB);

	// Initial Variable Used
	regAddr = 0;
	regData = 0;

	// Initial I2C Slave
	TWAR = (I2C_SLAVE_ADDR << 1) & 0xFE;    // Set I2C Address, Ignore I2C General Address 0x00
	TWDR = 0x00;            // Default Initial Value

	// Start Slave Listening: Clear TWINT Flag, Enable ACK, Enable TWI, TWI Interrupt Enable
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);

	InitRTOS(); //инициализация RTOS
	RunRTOS(RTOS_FREQ);	//запуск RTOS с частотой таймера 1000гц

	sei(); //разрешаем прерывания

	//режим работы
	SetTask(ApplyWorkMode);
	StartControlOnline(RTOS_FREQ);				// запуск задачи контроля состояния Онлайна
	SetTask(StartBeep);

	#if defined(TEST)
	testPWM = 128;
	testParrot = 50;
	testState = 0;
	SetTask(TestMotor);
	SetTask(TestServo);
	#endif


 	while(1)				// Основной цикл
 	{
 		TaskManager();		//вызов работы диспетчера задач RTOS
 		wdt_reset(); 		//глажу собаку
	}

	return 0;
}

