/*
 * UART.c
 *
 *  Created on: Jul 19, 2017
 *      Author: Minh Khang
 */

#include "../include.h"
#include "UART.h"
static char buff[16];

#ifdef non_UART_BUFFERED

typedef union{
    float numb;
    char temp[4];
}charFloat;

static charFloat save;
static void Reset_Buffer(char *pBuff)
{
    while(*pBuff!=0x00)
    {
        *pBuff=0;
        pBuff++;
    }
}

static void UART_Get_Buff(char *pBuff)
{
    static char c;
    static int16_t i=0;
    if (i==0)
    {
        Reset_Buffer(pBuff);
    }

    while(UARTCharsAvail(UART0_BASE))
    {
        c=UARTCharGet(UART0_BASE);
        *(pBuff+i)=c;
        i++;
    }
}

static void UART_ISR(void)
{
    //clear interrupt flag
    UARTIntClear(UART0_BASE, UARTIntStatus(UART0_BASE, true));
    UART_Get_Buff(&buff[0]);
    memcpy(save.temp, &buff, 4);
    UARTprintf("%u",save.numb);
}


extern void Config_UART(void)
{
    //config UART 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);
    //set the configuration for the UART - must be the same with receiver - using UART lib
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600 , UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    //Register Interrupt
    UARTIntRegister(UART0_BASE, &UART_ISR);
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    //config UART0
}

#endif

#ifdef UART_UNBUFFERED

int k;
static void UART_ISR(void)
{
    //clear interrupt flag
    UARTIntClear(UART0_BASE, UARTIntStatus(UART0_BASE, true));
    k=UARTgets(&buff[0], 100); //already disable echo function in line 179 of declaration
                               //return the number of character in the buffer
}

extern void Config_UART(void)
{
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //set the configuration for the UART -using uartstdio
    UARTStdioConfig(0, 115200, SysCtlClockGet());
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    //Register Interrupt
    UARTIntRegister(UART0_BASE, &UART_ISR);
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}
extern void PutBuffer(void)
{
    UARTwrite(&buff[0], k);
}
#endif

#ifdef UART_BUFFERED
//muon su dung ham interrupt co san cua UART Buffered phai thay doi dong 44 va 91 trong file
//tm4c123gh6pm_startup_ccs.c
void Config_UART(void)
{
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        //set the configuration for the UART -using uartstdio
        UARTStdioConfig(0, 115200, SysCtlClockGet());
        GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
        GPIOPinConfigure(GPIO_PA0_U0RX);
        GPIOPinConfigure(GPIO_PA1_U0TX);
        //Register Interrupt using Lib
        UARTEchoSet(true); //disable echo
}
#endif


