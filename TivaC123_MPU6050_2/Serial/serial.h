/*
 * serial.h
 *
 *  Created on: Aug 16, 2017
 *      Author: TAI
 */
#ifndef SERIAL_SERIAL_H_
#define SERIAL_SERIAL_H_
extern "C"{
extern void reverse(char *str, int len);
extern int intToStr(int x, char str[], int d);
extern void ftoa(float n, char *res, int afterpoint);
}


#endif /* SERIAL_SERIAL_H_ */
