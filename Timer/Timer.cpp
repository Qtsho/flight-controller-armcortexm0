/*
 * Timer.c
 *
 *  Created on: Aug 10, 2017
 *      Author: Quang Tien
 */

#include "Timer.h"
#include "../include.h"
volatile int flag = 1;

static void Timer_ISR(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
 flag = 1;

}
extern void Config_Timer(void){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC|TIMER_CFG_A_PWM);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 160000-1);//4ms/ thoi gian 1 clock = 160000
    TimerIntRegister(TIMER0_BASE, TIMER_A, &Timer_ISR);
IntEnable(INT_TIMER0A);
TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

}

