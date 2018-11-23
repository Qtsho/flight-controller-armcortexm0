/*
 * DMP.h
 *
 *  Created on: Aug 10, 2017
 *      Author: TAI
 */
#include "../include.h"
#ifndef DMP_DOC_DMP_H_
#define DMP_DOC_DMP_H_
extern "C"{
extern void Initialize_DMP(void);
extern void GetYawPitchRoll(void);
extern float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
}
#endif /* DMP_DOC_DMP_H_ */
