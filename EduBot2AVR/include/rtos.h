/*
 * eertos.h
 *
 *  Created on: 29 июня 2015 г.
 *      Author: воробьев
 */

#ifndef LIB_32M1_RTOS_H_
#define LIB_32M1_RTOS_H_

#include <avr/io.h>

//#define TIMERSERVICE_DISABLE_GLOBAL_INTERRUPT  //использование запрета глобальных прерываний в таймерной службе RTOS

//длина очереди задач
#define TaskQueueSize 20
//количество таймеров RTOS
#define MainTimerQueueSize 6

#ifndef F_CPU
#warning "F_CPU not defined for <32m1_rtos.h>"
#define F_CPU 16000000UL //тактовая частота
#endif

#define HI(x) ((x)>>8)	//старший разряд 8-битного числа
#define LO(x) ((x)&0xFF) //младший разряд 8-битного числа?0
#define TRUE 1
#define FALSE 0

typedef void (*TPTR)(void *data); //прототип функции РТОС, параметр функции указатель 2 байта

extern void RunRTOS(uint16_t freq);

extern void InitRTOS(void);
extern void Idle(void *data);

extern uint8_t SetTask(TPTR TS);
extern uint8_t SetTaskParam(TPTR TS, void *data);
extern uint8_t SetTimerTask(TPTR TS, const uint16_t time);
extern uint8_t SetTimerTaskParam(TPTR TS, const uint16_t time, void *data);

extern void TaskManager(void);
extern void TimerService(void);
//extern uint64_t GetTickCount(void);

//RTOS Errors Пока не используются.
//#define TaskSetOk			 'A'
//#define TaskQueueOverflow	 'B'
//#define TimerUpdated		 'C'
//#define TimerSetOk		 'D'
//#define TimerOverflow		 'E'

#endif /* LIB_32M1_RTOS_H_ */
