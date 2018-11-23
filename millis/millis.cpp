/*
 * millis.cpp
 *
 *  Created on: Aug 4, 2017
 *      Author: TAI
 */

#include "millis.h"
unsigned long SysCount;
void SysTickhandler(void){
    SysCount++;
}

void Config_SysTick(void){
    SysTickEnable();
    SysTickPeriodSet(SysCtlClockGet()/1000);
    SysTickIntRegister(&SysTickhandler);
    SysTickIntEnable();
}

