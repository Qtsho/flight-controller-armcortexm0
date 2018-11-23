/*
 * GPIO.cpp
 *
 *  Created on: Aug 4, 2017
 *      Author: TAI
 */
#include "GPIO.h"
void INTA_TOGGLE(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
    SysCtlDelay(2);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
    SysCtlDelay(2);
}
void Config_BLUE(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2,GPIO_PIN_2);
    SysCtlDelay(SysCtlClockGet()/2);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
    SysCtlDelay(SysCtlClockGet()/2);
}



