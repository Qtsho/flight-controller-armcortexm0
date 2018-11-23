/*
 * include.h
 *
 *  Created on: Aug 14, 2017
 *      Author: Quang Tien
 */

#ifndef INCLUDE_H_
#define INCLUDE_H_

//standardlib
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <iostream>
//software
#include "driverlib/gpio.h"
#include "driverlib/can.h"
#include "driverlib/mpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "driverlib/qei.h"
#include "driverlib/fpu.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/i2c.h"
#include "driverlib/udma.h"
#include "utils/cpu_usage.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
//hardware
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
//user library
//I2C
#include "I2Cdev/I2Cdev.h"
#include "MPU6050/MPU6050.h"
#include "HMC5883L/HMC5883L.h"
#include "DMP/DMP.h"
#include "millis/millis.h"
#include "GPIO/GPIO.h"
#include "MS5611/MS5611.h"
//PWM
#include "PWM/PWM.h"
//Timer
#include "Timer/Timer.h"
//ADC
#include "ADC/ADC.h"
//UART
#include "UART/UART.h"
#include "DMA/uartstdio.h"
#include "DMA/ustdlib.h"
#include "Serial/serial.h"
//DMA
#include "DMA/DMA.h"



//extern float angle_roll, angle_pitch;
#endif /* INCLUDE_H_ */
