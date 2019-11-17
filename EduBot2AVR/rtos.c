/*
 * eertos.c
 *
 *  Created on: 29 ���� 2015 �.
 *      Author: ��������
 */
#include "include/rtos.h"

#include <string.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include "include/uart.h"


//static uint64_t tickCount = 0;
//������� �����, ��������
typedef struct {
	TPTR gotoTask; //��������� �� ������
	void *data;  //��������� �������� ������
} TaskData;

//������� ����� RTOS
static TaskData taskQueue[TaskQueueSize+1]; //������� �����

typedef struct {
	TPTR gotoTask; //��������� �������� �� ������
	uint16_t time; //�������� � ��
	void *data;  //��������� �������� ������
} TimerData;

//������� RTOS
static TimerData mainTimer[MainTimerQueueSize];

#if defined(__AVR_ATmega32C1__) || \
	defined(__AVR_ATmega64C1__) || \
	defined(__AVR_ATmega16M1__) || \
	defined(__AVR_ATmega32M1__) || \
	defined(__AVR_ATmega64M1__) || \
	defined(__AVR_ATmega168__) || \
	defined(__AVR_ATmega168P__) || \
	defined(__AVR_ATmega328__) || \
	defined(__AVR_ATmega328P__)
ISR(TIMER2_COMPA_vect) //���������� ���������� ������� RTOS //������ �������� ����
{
	TimerService();
}
#endif

#if defined(__AVR_AT90CAN32__) || \
    defined(__AVR_AT90CAN64__) || \
    defined(__AVR_AT90CAN128__)
ISR(TIMER0_COMP_vect) //���������� ���������� ������� RTOS
{
	TimerService();
}
#endif

//rtos ������ ���������� �������
inline void RunRTOS(uint16_t freq)
{
	uint16_t Prescaler;

#if defined(__AVR_ATmega32C1__) || \
	defined(__AVR_ATmega64C1__) || \
	defined(__AVR_ATmega16M1__) || \
	defined(__AVR_ATmega32M1__) || \
	defined(__AVR_ATmega64M1__) || \
	defined(__AVR_ATmega168__) || \
	defined(__AVR_ATmega168P__) || \
	defined(__AVR_ATmega328__) || \
	defined(__AVR_ATmega328P__)

	TCCR2A = 2 << WGM20; 			//����� ctc, ������� ������������ ��� ���������� �������� ��������� OCR2A

	if (freq >= 10000)
	{
		Prescaler = 8;
		TCCR2B = 2 << CS00;	//������������ 8 (��� ����� ������ stepper controller)
	}
	else
	{
		Prescaler = 64;
		TCCR2B = 4 << CS00;	//������������ 64
	}

#endif
#if defined(__AVR_AT90CAN32__) || \
	defined(__AVR_AT90CAN64__) || \
	defined(__AVR_AT90CAN128__)

	Prescaler = 64;
	TCCR0A = 1 << WGM01 | 1 << CS00 | 1 << CS01;	//����� ctc, ������������ 64
#endif

	uint16_t TimerDivider = F_CPU/Prescaler/freq;

	TCNT2 = 0; //��������� �������� ��������
	OCR2A = TimerDivider; //������������� �������� � ������� ���������
	TIMSK2 = 1 << OCIE2A; //��������� ���������� �� ����������

	//sei(); ������ RTOS
	//if (UseUART())
		//printf_P(PSTR("RTOS %uHz\n"), freq);
}

//RTOS ����������. ������� ��������
inline void InitRTOS(void)
{
	TaskData* pTaskData = taskQueue;
	TimerData* pTimerData = mainTimer;
	uint8_t i;

	for(i = 0; i<(TaskQueueSize+1); i++)//�� ��� ������ ���������� Idle
	{
		pTaskData->gotoTask = NULL;
		pTaskData->data = NULL;
		pTaskData++;
	}

	for(i = 0; i < MainTimerQueueSize; i++)//�������� ��� �������
	{
		pTimerData->gotoTask = NULL;
		pTimerData->time = 0;
		pTimerData->data = NULL;
		pTimerData++;
	}

	//if (UseUART())
		//printf_P(PSTR("RTOS init.\n"));
}

//������ ��������� - ������� ����
inline void Idle(void *data)
{

}

//������� ��������� ������ � �������. ������������ �������� - ��������� �� �������
//��� ������������ �� ������� �������� RTOS, ��� �� ���� �������� ���������
uint8_t SetTask(TPTR TS){
	return SetTaskParam(TS, NULL);
}

//������� ��������� ������ � �������. ������������ �������� - ��������� �� ������� � ������������ �������� �������
uint8_t SetTaskParam(TPTR TS, void *data)
{
	uint8_t i = 0;
#ifndef TIMERSERVICE_DISABLE_GLOBAL_INTERRUPT
	uint8_t nointerrupted = 0;
#endif
	uint8_t res = 1;

#ifndef TIMERSERVICE_DISABLE_GLOBAL_INTERRUPT
	if(SREG & (1<<SREG_I)) //���� ���������� ���������, ���������
	{
		cli();
		nointerrupted = 1; //������ ����, ��� �� �� � ����������
	}
#endif

	TaskData *pTaskData = taskQueue;

	while(pTaskData->gotoTask) //������������� ������� ����� �� ������� ��������� ������
	{
		if(i == TaskQueueSize) //���� ������� �����������, �������
		{
			//if(nointerrupted) sei(); //���� �� �� � ����������, ��������� ����������
			res = 0;
			break; //�������
		}
		i++;
		pTaskData++;
	}

	if (res)
	{
		pTaskData->gotoTask = TS; //���������� ������ � �������
		pTaskData->data = data;

		pTaskData++;
		pTaskData->gotoTask = NULL; //���������� ��������� ������� ��������
		pTaskData->data = NULL;
	}

#ifndef TIMERSERVICE_DISABLE_GLOBAL_INTERRUPT
	if(nointerrupted) sei(); //�������� ����������, ���� �� � ����������� ����������
#endif

	return res;
}

uint8_t SetTimerTask(TPTR TS, const uint16_t time)
{
	return SetTimerTaskParam(TS, time, NULL);
}

//������� ��������� ������ �� �������.������������ ��������� - ��������� �� �������,
//����� �������� � ����� ���������� �������, ������������ ��������
uint8_t SetTimerTaskParam(TPTR TS, const uint16_t time, void *data)
{
#ifndef TIMERSERVICE_DISABLE_GLOBAL_INTERRUPT
	uint8_t nointerrupted = 0;
#endif
	uint8_t res = 1;

#ifndef TIMERSERVICE_DISABLE_GLOBAL_INTERRUPT
	if(SREG & (1<<SREG_I)) //�������� ������� ����������, �� ������� ����
	{
		cli();
		nointerrupted = 1;
	}
#endif

	if(time == 0) //���� ����� 0, ����� ����� ������ ������ � �������
	{
		res = SetTaskParam(TS, data);
		//if(nointerrupted) sei(); //��������� ����������
	}
	else
	{
		TimerData* pTimerData = mainTimer;
		TimerData* freeTimer = NULL;

		for(uint8_t i = 0; i < MainTimerQueueSize; i++) //�������� �� ���� ������� ��������
		{
			if ((pTimerData->gotoTask == TS) && (pTimerData->data == data)) //���� ��� ���� ������ � ����� ������� � ����������
			{
				freeTimer = pTimerData; //���������� ����� ������ � ������� �� �����
				break; //�������
			}
			else
			{
				//�� ������, ���� � ������ �������� ��� ������ TS, ���������� ����� ������� �� �������� �������
				if((pTimerData->gotoTask == NULL) && (freeTimer == NULL))
					freeTimer = pTimerData;
			}
			pTimerData++;
		}

		if (freeTimer) //���� � ������� ���� ��������� ������
		{
			//������ ����� ������
			freeTimer->gotoTask = TS; //�������� ���� �������� ������
			freeTimer->time = (time - 1); //���� �������� �������
			freeTimer->data = data;
			//if(nointerrupted) sei(); //���� �� �� � ����������, ��������� ����������
		}
		else
			res = 0; //��� ������� �������������, ���������� ������ �� �������
	}

#ifndef TIMERSERVICE_DISABLE_GLOBAL_INTERRUPT
	if(nointerrupted) sei(); //��������� ����������
#endif

	return res;
}

// ��������� ����� ��. �������� �� ������� ������ � ���������� �� ����������.
inline void TaskManager(void)
{
	TaskData *pTaskData = taskQueue;

	cli(); //��������� ����������
	TPTR gotoTask = pTaskData->gotoTask; //����� ������ ������ �� �������
	void *data = pTaskData->data; //��������� �� ��������� ������
	sei(); //��������� ����������


	if(gotoTask) //���� ������ �� NULL
	{
		cli();
		while(pTaskData->gotoTask) //���� ��� �� ����, ������� ��� �������
		{
			//memcpy(pTaskData, pTaskData+1, sizeof(TaskData)); //���������� �������� �������� ������
			*pTaskData = *(pTaskData+1);
			pTaskData++;
		}
		sei();

		gotoTask(data); //��������� � ������ � �������� �� ��������
	}
	else
		Idle(NULL); //��������� �� ��������� ������ Idle (������� ����)
}

//������ �������� ����. ���������� �� ���������� ��� � 1 ��.
inline void TimerService(void)
{
	TimerData *pTimerData = mainTimer;

	for(uint8_t i = 0; i < MainTimerQueueSize; i++) //�������� ��� �������
	{
		if(pTimerData->gotoTask) //���� ���� � ������� ������
		{
			if(pTimerData->time) //���� ������ �� 0, ������ ��� ��������
				pTimerData->time--; //��������� ����� � ������, ���� ��� �� �����
			else
			{
				SetTaskParam(pTimerData->gotoTask, pTimerData->data); //���� ������ ����� �� ����, ���������� � ������� �����
				pTimerData->gotoTask = NULL; //�������������� ������ ��������
			}
		}

		pTimerData++; //��������� � ���������� �������
	}
}
