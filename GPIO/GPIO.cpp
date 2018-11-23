/*
 * GPIO.cpp
 *
 *  Created on: Aug 4, 2017
 *      Author: TAI
 */
#include "GPIO.h"
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    GPIOIntClear(GPIO_PORTF_BASE, GPIOIntStatus(GPIO_PORTF_BASE, true));
    mpuInterrupt = true;
}
void INTA_SETUP(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_1 , GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_RISING_EDGE);
    GPIOIntRegister(GPIO_PORTF_BASE, &dmpDataReady);
    GPIOIntEnable(GPIO_PORTF_BASE,GPIO_INT_PIN_1);
    IntEnable(INT_GPIOF);

}
void INTA_TOGGLE(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
    SysCtlDelay(2);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
    SysCtlDelay(2);
}



