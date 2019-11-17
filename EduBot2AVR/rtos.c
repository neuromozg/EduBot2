/*
 * eertos.c
 *
 *  Created on: 29 июня 2015 г.
 *      Author: воробьев
 */
#include "include/rtos.h"

#include <string.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include "include/uart.h"


//static uint64_t tickCount = 0;
//очереди задач, таймеров
typedef struct {
	TPTR gotoTask; //указатель на задачу
	void *data;  //указатель параметр задачи
} TaskData;

//очередь задач RTOS
static TaskData taskQueue[TaskQueueSize+1]; //очередь задач

typedef struct {
	TPTR gotoTask; //указатель перехода на задачу
	uint16_t time; //выдержка в мс
	void *data;  //указатель параметр задачи
} TimerData;

//таймеры RTOS
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
ISR(TIMER2_COMPA_vect) //обработчик прерывания таймера RTOS //служба таймеров ядра
{
	TimerService();
}
#endif

#if defined(__AVR_AT90CAN32__) || \
    defined(__AVR_AT90CAN64__) || \
    defined(__AVR_AT90CAN128__)
ISR(TIMER0_COMP_vect) //обработчик прерывания таймера RTOS
{
	TimerService();
}
#endif

//rtos запуск системного таймера
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

	TCCR2A = 2 << WGM20; 			//режим ctc, регистр сбрасывается при достижении регистра сравнения OCR2A

	if (freq >= 10000)
	{
		Prescaler = 8;
		TCCR2B = 2 << CS00;	//предделитель 8 (для платы Давида stepper controller)
	}
	else
	{
		Prescaler = 64;
		TCCR2B = 4 << CS00;	//предделитель 64
	}

#endif
#if defined(__AVR_AT90CAN32__) || \
	defined(__AVR_AT90CAN64__) || \
	defined(__AVR_AT90CAN128__)

	Prescaler = 64;
	TCCR0A = 1 << WGM01 | 1 << CS00 | 1 << CS01;	//режим ctc, предделитель 64
#endif

	uint16_t TimerDivider = F_CPU/Prescaler/freq;

	TCNT2 = 0; //начальное значение счетчика
	OCR2A = TimerDivider; //устанавливаем значение в регистр сравнения
	TIMSK2 = 1 << OCIE2A; //разрешаем прерывания по совпадению

	//sei(); запуск RTOS
	//if (UseUART())
		//printf_P(PSTR("RTOS %uHz\n"), freq);
}

//RTOS подготовка. Очистка очередей
inline void InitRTOS(void)
{
	TaskData* pTaskData = taskQueue;
	TimerData* pTimerData = mainTimer;
	uint8_t i;

	for(i = 0; i<(TaskQueueSize+1); i++)//во все задачи записываем Idle
	{
		pTaskData->gotoTask = NULL;
		pTaskData->data = NULL;
		pTaskData++;
	}

	for(i = 0; i < MainTimerQueueSize; i++)//обнуляем все таймеры
	{
		pTimerData->gotoTask = NULL;
		pTimerData->time = 0;
		pTimerData->data = NULL;
		pTimerData++;
	}

	//if (UseUART())
		//printf_P(PSTR("RTOS init.\n"));
}

//пустая процедура - простой ядра
inline void Idle(void *data)
{

}

//функция установки задачи в очередь. Передаваемый параметр - указатель на функцию
//для совметимости со старыми версиями RTOS, где не было передачи параметра
uint8_t SetTask(TPTR TS){
	return SetTaskParam(TS, NULL);
}

//функция установки задачи в очередь. Передаваемый параметр - указатель на функцию и двухбайтовый параметр функции
uint8_t SetTaskParam(TPTR TS, void *data)
{
	uint8_t i = 0;
#ifndef TIMERSERVICE_DISABLE_GLOBAL_INTERRUPT
	uint8_t nointerrupted = 0;
#endif
	uint8_t res = 1;

#ifndef TIMERSERVICE_DISABLE_GLOBAL_INTERRUPT
	if(SREG & (1<<SREG_I)) //если прерывания разрешены, запрещаем
	{
		cli();
		nointerrupted = 1; //ставим флаг, что мы не в прерывании
	}
#endif

	TaskData *pTaskData = taskQueue;

	while(pTaskData->gotoTask) //просматриваем очередь задач на предмет свободной ячейки
	{
		if(i == TaskQueueSize) //если очередь переполнена, выходим
		{
			//if(nointerrupted) sei(); //если мы не в прерывании, разрешаем прерывания
			res = 0;
			break; //выходим
		}
		i++;
		pTaskData++;
	}

	if (res)
	{
		pTaskData->gotoTask = TS; //записываем задачу в очередь
		pTaskData->data = data;

		pTaskData++;
		pTaskData->gotoTask = NULL; //записываем следующей задачей заглушку
		pTaskData->data = NULL;
	}

#ifndef TIMERSERVICE_DISABLE_GLOBAL_INTERRUPT
	if(nointerrupted) sei(); //включаем прерывания, если не в обработчике прерываний
#endif

	return res;
}

uint8_t SetTimerTask(TPTR TS, const uint16_t time)
{
	return SetTimerTaskParam(TS, time, NULL);
}

//функция установки задачи по таймеру.передаваемые параметры - указатель на функцию,
//время выдержки в тиках системного таймера, двухбайтовый параметр
uint8_t SetTimerTaskParam(TPTR TS, const uint16_t time, void *data)
{
#ifndef TIMERSERVICE_DISABLE_GLOBAL_INTERRUPT
	uint8_t nointerrupted = 0;
#endif
	uint8_t res = 1;

#ifndef TIMERSERVICE_DISABLE_GLOBAL_INTERRUPT
	if(SREG & (1<<SREG_I)) //проверка запрета прерываний, см функцию выше
	{
		cli();
		nointerrupted = 1;
	}
#endif

	if(time == 0) //если время 0, тогда сразу ставим задачу в очередь
	{
		res = SetTaskParam(TS, data);
		//if(nointerrupted) sei(); //разрешаем прерывания
	}
	else
	{
		TimerData* pTimerData = mainTimer;
		TimerData* freeTimer = NULL;

		for(uint8_t i = 0; i < MainTimerQueueSize; i++) //проходим по всей очереди таймеров
		{
			if ((pTimerData->gotoTask == TS) && (pTimerData->data == data)) //если уже есть запись с таким адресом и параметром
			{
				freeTimer = pTimerData; //запоминаем адрес ячейки и выходим из цикла
				break; //выходим
			}
			else
			{
				//на случай, если в списке таймеров нет задачи TS, запоминаем адрес первого не занятого таймера
				if((pTimerData->gotoTask == NULL) && (freeTimer == NULL))
					freeTimer = pTimerData;
			}
			pTimerData++;
		}

		if (freeTimer) //если в системе есть свободный таймер
		{
			//задаем новый таймер
			freeTimer->gotoTask = TS; //заполяем поле перехода задачи
			freeTimer->time = (time - 1); //поле выдержки времени
			freeTimer->data = data;
			//if(nointerrupted) sei(); //если мы не в прерывании, разрешаем прерывания
		}
		else
			res = 0; //все таймеры задействованы, установить таймер не удалось
	}

#ifndef TIMERSERVICE_DISABLE_GLOBAL_INTERRUPT
	if(nointerrupted) sei(); //разрешаем прерывания
#endif

	return res;
}

// Диспетчер задач ОС. Выбирает из очереди задачи и отправляет на выполнение.
inline void TaskManager(void)
{
	TaskData *pTaskData = taskQueue;

	cli(); //запрещаем прерывания
	TPTR gotoTask = pTaskData->gotoTask; //берем первую задачу из очереди
	void *data = pTaskData->data; //указатель на параметры задачи
	sei(); //разрешаем прерывания


	if(gotoTask) //если задача не NULL
	{
		cli();
		while(pTaskData->gotoTask) //если что то есть, двигаем всю очередь
		{
			//memcpy(pTaskData, pTaskData+1, sizeof(TaskData)); //перемещаем значение соседней ячейки
			*pTaskData = *(pTaskData+1);
			pTaskData++;
		}
		sei();

		gotoTask(data); //переходим к задаче и передаем ей параметр
	}
	else
		Idle(NULL); //переходим на обработку задачи Idle (простой ядра)
}

//служба таймеров ядра. вызывается из прерывания раз в 1 мс.
inline void TimerService(void)
{
	TimerData *pTimerData = mainTimer;

	for(uint8_t i = 0; i < MainTimerQueueSize; i++) //проходим все таймеры
	{
		if(pTimerData->gotoTask) //если есть в таймере задача
		{
			if(pTimerData->time) //если таймер не 0, делаем еще итерацию
				pTimerData->time--; //уменьшаем число в ячейке, если еще не конец
			else
			{
				SetTaskParam(pTimerData->gotoTask, pTimerData->data); //если таймер дошел до нуля, отправляем в очередь задач
				pTimerData->gotoTask = NULL; //освободившуюся ячейку обнуляем
			}
		}

		pTimerData++; //переходим к следующему таймеру
	}
}
