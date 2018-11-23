/*
 * PWM.h
 *
 *  Created on: Aug 8, 2017
 *      Author: Quang Tien
 */

#ifndef PWM_PWM_H_
#define PWM_PWM_H_



#define PWM_FREQUENCY 50
extern "C" {
extern void Config_PWM(void);
extern void esc2Write(unsigned int ms2);
extern void esc1Write(unsigned int ms1);
extern void esc3Write(unsigned int ms3);
extern void esc4Write(unsigned int ms4);
}
#endif /* PWM_PWM_H_ */
