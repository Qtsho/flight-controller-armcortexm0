/*
 * PWM.c
 *
 *  Created on: Aug 8, 2017
 *      Author: Quang Tien
 */

#include "PWM.h"
#include "../include.h"
static volatile uint32_t ui32Load;
void delayMS(int ms) {
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms );
}

extern void esc1Write(unsigned int ms1)
{
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ms1 * ui32Load /10000);

}
extern void esc2Write(unsigned int ms2)
{
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ms2 * ui32Load /10000);
}
extern void esc3Write(unsigned int ms3)
{
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ms3 * ui32Load /10000);
}
extern void esc4Write(unsigned int ms4)
{
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ms4 * ui32Load /10000);
}
extern  void Config_PWM (void)
    {
    //    volatile uint32_t ui32Load;
        volatile uint32_t ui32PWMClock;
        volatile uint32_t ui8Adjust;
        ui8Adjust = 0;
        // Thiet lap clock he thong
         SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
        // Cau hinh tan so PWM
         SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
        // Kich hoat chan PW1, port D va Fs
         SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

         SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
         SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
         SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

         GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
         GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);//Cau hinh port D la pwm
         GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
             GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);
         GPIOPinConfigure(GPIO_PD0_M1PWM0);  //Cau hinh chan PD0 la chan PWM
         GPIOPinConfigure(GPIO_PD1_M1PWM1);
         GPIOPinConfigure(GPIO_PE4_M1PWM2);//Cau hinh chan PE4 la chan PWM
         GPIOPinConfigure(GPIO_PE5_M1PWM3);//Cau hinh chan PE5 la chan PWM

        //Dinh nghia lai phan cung
        HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
        HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
        HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

        //Cau hinh chan PF4 va PF0 lam nut nhan
         GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
         GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

        //Tinh clock PWM
        ui32PWMClock = SysCtlClockGet() /64;
        ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
        //Cau hinh PWM
        PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
        PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
        PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);    //Tao chu ky PWM
        PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, ui32Load);    //Tao chu ky PWM

         PWMPulseWidthSet(PWM1_BASE,PWM_OUT_0,ui8Adjust * ui32Load /1000);
         PWMPulseWidthSet(PWM1_BASE,PWM_OUT_1,ui8Adjust * ui32Load /1000);
         PWMPulseWidthSet(PWM1_BASE,PWM_OUT_2,ui8Adjust * ui32Load /1000);
         PWMPulseWidthSet(PWM1_BASE,PWM_OUT_3,ui8Adjust * ui32Load /1000);
         PWMOutputState(PWM1_BASE,PWM_OUT_0_BIT, true); //Xuat output ra PWM
         PWMOutputState(PWM1_BASE,PWM_OUT_1_BIT, true); //Xuat output ra PWM
         PWMOutputState(PWM1_BASE,PWM_OUT_3_BIT, true);
         PWMOutputState(PWM1_BASE,PWM_OUT_2_BIT, true);
         PWMGenEnable(PWM1_BASE, PWM_GEN_0); //Kich hoat PWM
         PWMGenEnable(PWM1_BASE, PWM_GEN_1);

}


