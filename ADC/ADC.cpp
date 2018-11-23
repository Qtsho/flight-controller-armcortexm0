/*
 * ADC.c
 *
 *  Created on: Jul 29, 2017
 *      Author: Quang Tien
 */
#include "ADC.h"
#include "../include.h"

static void ADC_ISR(void);
extern void Config_ADC(void)
{
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);// port E PE3
SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); //configre module 0

GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
ADCHardwareOversampleConfigure(ADC0_BASE, 64); // tinh trung binh de loc nhieu

// sample sequencer
ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0); // sequencer
ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0); // configure step adc0 base module adc nafo?
ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH0);
ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH0);
ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH0|ADC_CTL_END|ADC_CTL_IE);

ADCSequenceEnable(ADC0_BASE, 1);

ADCIntRegister(ADC0_BASE, 1, &ADC_ISR); // lay mau xong se interrupt
ADCIntEnable(ADC0_BASE, 1);

IntMasterEnable();

}

static void ADC_ISR(void)
{
    // lay du lieu tu FIFO
    ADCIntClear(ADC0_BASE, 1);
    uint32_t Data[4];
    ADCSequenceDataGet(ADC0_BASE, 1, (uint32_t *)&Data);// ep kieu data thanh uint32 roi tro vao mang data
    // du lieu tra ra data la dang so
    float Voltage;
    Voltage = (float)((Data[0]+Data[1]+Data[2]+Data[3])/4)*5/4096;
 }
