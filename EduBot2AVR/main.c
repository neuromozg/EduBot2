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
#define RTOS_FREQ 1000U // ������� ������ ������� RTOS
#define WAIT_ONLINE_SEC 3 		//���������� ������ �� ����� ������� ������ ������� ������ ������

#define LED_INF_BIT PB0 //����� ��������������� ����������
#define LED_INF_DDR DDRB
#define LED_INF_PORT PORTB

//����� ������ A
//���
#define MOTOR_A_PWM_BIT PD6
#define MOTOR_A_PWM_DDR DDRD
#define MOTOR_A_PWM_PORT PORTD
#define MOTOR_A_PWM_PIN PIND

//����������� ��������
#define MOTOR_A_CW_BIT PC2
#define MOTOR_A_CW_DDR DDRC
#define MOTOR_A_CW_PORT PORTC
#define MOTOR_A_CW_PIN PINC

//���� ������� ����
#define MOTOR_A_FG_BIT PD2
#define MOTOR_A_FG_DDR DDRD
#define MOTOR_A_FG_PORT PORTD
#define MOTOR_A_FG_PIN PIND

#define MOTOR_A_PWM_TIMER_REG OCR0A

//����� ������ B
//���
#define MOTOR_B_PWM_BIT PD5
#define MOTOR_B_PWM_DDR DDRD
#define MOTOR_B_PWM_PORT PORTD
#define MOTOR_B_PWM_PIN PIND

//����������� ��������
#define MOTOR_B_CW_BIT PD4
#define MOTOR_B_CW_DDR DDRD
#define MOTOR_B_CW_PORT PORTD
#define MOTOR_B_CW_PIN PIND

//���� ������� ����
#define MOTOR_B_FG_BIT PD3
#define MOTOR_B_FG_DDR DDRD
#define MOTOR_B_FG_PORT PORTD
#define MOTOR_B_FG_PIN PIND

#define MOTOR_B_PWM_TIMER_REG OCR0B

//�����0
#define SERVO0_BIT PB1
#define SERVO0_DDR DDRB
#define SERVO0_PORT PORTB
#define SERVO0_PIN PINB

#define SERVO0_PWM_TIMER_REG OCR1A

//�����1
#define SERVO1_BIT PB2
#define SERVO1_DDR DDRB
#define SERVO1_PORT PORTB
#define SERVO1_PIN PINB

#define SERVO1_PWM_TIMER_REG OCR1B

//�����
#define BUZZER_BIT PC3
#define BUZZER_DDR DDRC
#define BUZZER_PORT PORTC
#define BUZZER_PIN PINC

//������
#define BUTTON_BIT PC0
#define BUTTON_DDR DDRC
#define BUTTON_PORT PORTC
#define BUTTON_PIN PINC

//������ ������ �������
#define MOTOR_MODE_PWM 0 //��������� ����������� �������� ����� ������� PWM
#define MOTOR_MODE_PID 1 //��������� ����������� ����� ��� ���������

#define SERVO_BASE_TIMER 250 //�������� ����� ������� ������������ ������� 1000��
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

//------��������� ������� ����������� � EEPROM----------------------------
typedef struct {
	uint8_t workMode; //����� ������ �����������
	float kP;  							// ���������������� �����������
	int8_t kPint;
	float kI;  							// ������������ �����������
	int8_t kIint;
	float kD;  							// ���������������� �����������
	int8_t kDint;
	int16_t limitSum;					// ����������� ������������ �����
	uint16_t timePID;					// ����� ������� ���-����������
} WorkParams;

typedef void (*pSetPIDMotor)(int16_t);

typedef struct {
	char name; //��� ������
	volatile uint8_t* timerPWMReg;		//����� �������
	volatile uint8_t* portCw; //���� ������ ����������� ��������
	volatile uint8_t* pinCw; //��� ������ ����������� ��������
	uint8_t bitCwMask; //����� ��� ������ ����������� ��������
	volatile uint8_t* pinFg; //��� ����� ������� �����
	uint8_t bitFgMask; //����� ��� ����� ������� �����
	uint8_t dir; //����������� �������� ��� ������� ����� i2c
	int32_t odomCount; //������� ���������
	int32_t odomCountOld;
	int16_t setParrot;	//�������� �������
	int16_t integralSum; //������������ �����
	int16_t oldErrorParrot;	//���������� �������� ��������
	int16_t currentParrot; 	//������� �������
	float pParrot; //���������������� ������������
	float iParrot; //������������ ������������
	float dParrot; //���������������� ������������
	int16_t resPID; //��������� ���
	pSetPIDMotor SetPIDMotor;
} MotorData;

WorkParams workParams;

MotorData motorA;
MotorData motorB;

uint8_t regAddr; // Store the Requested Register Address
uint8_t regData; // Store the Register Address Data

uint8_t tagOnline; //������� �������
uint8_t countOffline = 0; //������� ���������
uint16_t pauseIndicationOnline = 100; //������� � �� ��� �������� ��������� ��� �������
uint16_t pauseIndicationOffline = 1000; //������� � �� ��� �������� ��������� ��� ��������

uint8_t tagBeep; //�������, ��� ������ �����

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

//������ RTOS
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
                regAddr = TWDR; // �������� ����� ��������
                i2c_state = 1;
            } else {
                regData = TWDR; // �������� ������
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

//���������� �� ��������� ��������� ����� �� ������� ���� A
ISR(INT0_vect)
{
	OdometrProcess(&motorA);
}

//���������� �� ��������� ��������� ����� �� ������� ���� B
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
			regData = 0x2A; //����� �� ������� ������ �����, ��������� � ����� ������
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
				//������ ������� � ����������� �� ����������� ��������
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
				//������ ������� � ����������� �� ����������� ��������
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
				//������ ����������� ��������
				if (motorA.dir)
					(*motorA.portCw) |= motorA.bitCwMask;
				else
					(*motorA.portCw) &= ~motorA.bitCwMask;

				//������ �������� ��������
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
				//������ ����������� ��������
				if (motorB.dir)
					(*motorB.portCw) |= motorB.bitCwMask;
				else
					(*motorB.portCw) &= ~motorB.bitCwMask;

				//������ �������� ��������
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

void ServoSetPos(volatile uint8_t* timerReg, uint8_t pos) //������ ��������� ������������
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
	data->odomCount = 0; //������� ���������
	data->odomCountOld = 0;
	data->setParrot = 0;	//�������� �������
	data->integralSum = 0; //������������ �����
	data->oldErrorParrot = 0;	//���������� �������� ��������
	data->currentParrot = 0; 	//������� �������
	data->pParrot = 0; //���������������� ������������
	data->iParrot = 0; //������������ ������������
	data->dParrot = 0; //���������������� ������������
	data->resPID = 0; //��������� ���
	data->dir = 0; //����������� ��������
}

void OdometrProcess(MotorData *data)
{
	if (*data->pinCw & data->bitCwMask) //� ����������� �� ����������� ��������
		data->odomCount++; //����������� ������� ��������
	else
		data->odomCount--; //��������� ������� ��������
}

void CalcPID(MotorData *data)
{
	int16_t errorParrot; //������ ��������
	int32_t odomCount;
	int16_t setParrot;
	//uint16_t stepIntegralDecrement;

	cli(); //�������� ����������� ������, ����������� �� ����������
	odomCount = data->odomCount; //������� ��������� �������� (���������� �� �������)
	setParrot = data->setParrot; //�������, �������� �������� �������� (�������� �� i2c)
	sei();

	data->currentParrot = odomCount - data->odomCountOld;		//���������� �������� ������� (��������)
	data->odomCountOld = odomCount;

	errorParrot = setParrot - data->currentParrot; //���������� ������

	data->integralSum += errorParrot;          // ���������� ��������� (������������ ����� ������)

	/*
	//���������� ��������� ����� ����������� ������
	Parms->integralBuff[Parms->integralCount] = errorParrot; //��������� ������ � ������
	Parms->integralCount++;
	if (Parms->integralCount > (IntegralBuffelLength-1)) //�������� �� ������������ ��������
		Parms->integralCount = 0;

	//���������� ���������
	Parms->integralSum = 0;
	for (uint8_t i = 0; i<IntegralBuffelLength; i++)
		Parms->integralSum += Parms->integralBuff[i]; //��������� ������ ���������� �������
	*/

	if (data->integralSum > workParams.limitSum) //������������ ��������� � �������� �������� �������
		data->integralSum = workParams.limitSum;
	else
	{
		if (data->integralSum < -workParams.limitSum)
			data->integralSum = -workParams.limitSum;
	}

	//�������� �������, ��������� ������������ ����� �� 0
	//�������, � ������� ����� ����� 0, ����� ����������� �.�. ���� 0, � ������������ ����� ��������� � ������ ���� �������� � ��������� ���
	if ((setParrot == 0) && (data->currentParrot == 0) && (data->integralSum != 0))
	{
		if (data->integralSum > 0)
			data->integralSum --;
		else
			if (data->integralSum < 0)
				data->integralSum ++;
	}

	//���������� ��������� ��� �������� ������� ��������,
	//������� ��� ����, ����� ���������� �������� ������������ ������������, ����� ��������� ����������
	//� ��� ��������� ��� �� ��������, �� ���. ����� ������ �� ���
//	if ((data->setParrot == 0) && (data->currentParrot == 0) && (data->integralSum != 0))
//	{
//		stepIntegralDecrement = workParams.timePID >> 2; //��� ���������� ���������, ����� ������� ��� ����� �� 4, ���� ����������� ���
//		if (stepIntegralDecrement == 0)
//			stepIntegralDecrement = 1;
//
//		if (data->integralSum > 0)
//		{
//			if (data->integralSum > stepIntegralDecrement) //���������� ������������� �����
//				data->integralSum -= stepIntegralDecrement;
//			else
//				data->integralSum = 0;
//		}
//		else
//		{
//			if (data->integralSum < -stepIntegralDecrement) //���������� ������������� �����
//				data->integralSum += stepIntegralDecrement;
//			else
//				data->integralSum = 0;
//		}
//	}

	data->pParrot = workParams.kP * errorParrot;    // ���������������� ������������
	data->iParrot = workParams.kI * data->integralSum;    // ������������ ������������
	data->dParrot = workParams.kD * (errorParrot - data->oldErrorParrot); //���������������� ������������
	data->oldErrorParrot = errorParrot; //�������� ������
	data->resPID = data->pParrot + data->iParrot + data->dParrot; //���������� ���������� ���

	SetMotorDirPWM(data, data->resPID);

//	printf_P(PSTR("odo:%d set:%d curr:%d P:%.2f I:%.2f D:%.2f res: %d\n"), (int)odomCount, (int)setParrot, (int)motorA.currentParrot,
//			(float)motorA.pParrot, (float)motorA.iParrot, (float)motorA.dParrot, (int)motorA.resPID); //��������

}

void SetMotorDir(MotorData *data, uint8_t dir)
{
	//������ ����������� ��������
	if (dir)
	{
		(*data->portCw) |= data->bitCwMask; //������ ���������� ��������
	}
	else
	{
		(*data->portCw) &= ~data->bitCwMask; //������ ���������� ��������
	}
}

void SetMotorPWM(MotorData *data, uint8_t pwm)
{
	pwm = ~pwm; //����������� PWM ��� ������� 25D

	*data->timerPWMReg = pwm; //������ �������� ��� � ������� �������
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
		SetTaskParam((TPTR)CalcPID, &motorA); //������ ������ ���������� ��� 1-�� ������
		SetTaskParam((TPTR)CalcPID, &motorB);

		SetTimerTask(UpdatePID, workParams.timePID); //��������� ����� ������
	}
}

void CalcParrot(MotorData *data)
{
	int32_t odomCount;


	cli(); //�������� ����������� ������, ����������� �� ����������
	odomCount = data->odomCount; //������� ��������� �������� (���������� �� �������)
	sei();

	data->currentParrot = odomCount - data->odomCountOld;		//���������� �������� ������� (��������)
	data->odomCountOld = odomCount;


	//data->MotorGo(Parms->resPWM); //������ ��� � ����������� �������� �� ������� ������
	//SetTaskParam(&Parms->MotorGo, Parms->resPWM); //��������� ��������� ��������� ���������� ��� � ����������� �������� �� ������� ������
}

void UpdateParrot(void *data)
{
	if (workParams.workMode == MOTOR_MODE_PWM)
	{
		SetTaskParam((TPTR)CalcParrot, &motorA); //������ ������ ���������� ��������
		SetTaskParam((TPTR)CalcParrot, &motorB);

		SetTimerTask(UpdateParrot, workParams.timePID); //��������� ����� ������
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
	LED_INF_PORT ^= 1<<LED_INF_BIT; //����������� ���������

	if (tagOnline)
		SetTimerTask(IndicationOnLine, pauseIndicationOnline); //���� ������ �� ������ ������
	else
		SetTimerTask(IndicationOnLine, pauseIndicationOffline); //���� ������� ������ ��� � �������
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
			SetTask(OffLineEvent); //�������� ���������� ������ �������
		}
	}

	SetTimerTask(ControlOnline, pauseIndicationOffline); //���������� ������ ����� 1 ���
}

void AcceptedOnlineLabel(void *data)
{
	countOffline = 0; //����� �������� ���������� �����
	if (tagOnline == FALSE)
	{
		tagOnline = TRUE; //������
		SetTask(OnLineEvent); //�������� ���������� ������ �������
	}
}

void StartControlOnline(const uint16_t freqRTOS)
{
	//��������� ������� ������� ���������� � ����������� �� ������� RTOS
	pauseIndicationOnline = freqRTOS >> 3; //������� ��������� ��� ������� ����� 1/8 ���
	pauseIndicationOffline = freqRTOS; //������� ��������� ��� �������� ����� 1 ���

	SetTask(ControlOnline); //������ ������ �������� ���������� � �� �������� ������ (Online)
	SetTask(IndicationOnLine); //������ ������ ��������� ���������� (������ �����������)
}

void OffLineEvent(void *data)
{
	//������� �������
	StopMotor(&motorA);
	StopMotor(&motorB);

	//����� � ������� ���������
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
	wdt_enable(WDTO_500MS); //��������� ���������� ������

	//�������������� ������ 0 ��� ��������� ��� ���������� ��������� ���������
	TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00); //���������� ���� � �������, ����� FastPWM
	TCCR0B = (3 << CS00); //�������� 64, ��� 976��

	//�������������� ������ 1 ��� ��������� ��� ���������� �������������
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11); //���������� ���� � �������, ����� FastPWM ����� ������� �� �������� OCR1 ����� 1
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (3 << CS00); //�������� 64
	ICR1 = 4999; //��� 20��

	SERVO0_PWM_TIMER_REG = SERVO_BASE_TIMER + SERVO_MED; //����� � ������� ���������
	SERVO1_PWM_TIMER_REG = SERVO_BASE_TIMER + SERVO_MED;

	//����������� ������ � ���������� ���������� ��� ������ ������� �����
	EICRA = (3 << ISC10) | (3 << ISC00); //����� ���������� �����
	EIMSK = (1 << INT1) | (1 << INT0); //��������� ���������� ��� ������ INT1, INT0

	//������������� ����� IO
	//����������
	LED_INF_DDR |= (1 << LED_INF_BIT);

	//������ ��� � ���������� �������� ������ �
	MOTOR_A_PWM_DDR |= (1 << MOTOR_A_PWM_BIT);
	MOTOR_A_CW_DDR |= (1 << MOTOR_A_CW_BIT);

	//������ ��� � ���������� �������� ������ B
	MOTOR_B_PWM_DDR |= (1 << MOTOR_B_PWM_BIT);
	MOTOR_B_CW_DDR |= (1 << MOTOR_B_CW_BIT);

	//������������� ��������� ��� ����� ������� ���� (�������, � ��������� ������ ��������)
	//MOTOR_A_FG_PORT |= (1 << MOTOR_A_FG_BIT);
	//MOTOR_B_FG_PORT |= (1 << MOTOR_B_FG_BIT);

	//����
	BUZZER_DDR |= (1 << BUZZER_BIT);

	//������ ������������
	SERVO0_DDR |= (1 << SERVO0_BIT);
	SERVO1_DDR |= (1 << SERVO1_BIT);

	OCR0A = 0xFF; //�� ������ ������ ��� �� ������, ������ ����
	OCR0B = 0xFF;

	//����� ������
	workParams.workMode = MOTOR_MODE_PWM;

	//���������� ������
	workParams.kP = 0.5;  							// ���������������� �����������
	workParams.kPint = 5;
	workParams.kI = 0.5;  							// ������������ �����������
	workParams.kIint = 5;
	workParams.kD = 0;  							// ���������������� �����������
	workParams.kDint = 0;
	workParams.limitSum = 500;					// ����������� ������������ �����
	workParams.timePID = 100;

	tagBeep = FALSE;
	tagOnline = FALSE;

	//��������� ���������
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

	InitRTOS(); //������������� RTOS
	RunRTOS(RTOS_FREQ);	//������ RTOS � �������� ������� 1000��

	sei(); //��������� ����������

	//����� ������
	SetTask(ApplyWorkMode);
	StartControlOnline(RTOS_FREQ);				// ������ ������ �������� ��������� �������
	SetTask(StartBeep);

	#if defined(TEST)
	testPWM = 128;
	testParrot = 50;
	testState = 0;
	SetTask(TestMotor);
	SetTask(TestServo);
	#endif


 	while(1)				// �������� ����
 	{
 		TaskManager();		//����� ������ ���������� ����� RTOS
 		wdt_reset(); 		//����� ������
	}

	return 0;
}

