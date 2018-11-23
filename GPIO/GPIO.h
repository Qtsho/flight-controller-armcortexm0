/*
 * GPIO.h
 *
 *  Created on: Aug 4, 2017
 *      Author: TAI
 */

#ifndef GPIO_GPIO_H_
#define GPIO_GPIO_H_
#include "../include.h"
extern "C"{
extern void INTA_SETUP(void);
extern void INTA_TOGGLE(void);
extern volatile bool mpuInterrupt;
}
#endif /* GPIO_GPIO_H_ */
