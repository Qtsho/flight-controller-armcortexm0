/*
 * DMP.cpp
 *
 *  Created on: Aug 10, 2017
 *      Author: TAI
 */

#include "DMP.h"
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
MPU6050 mpu;
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal; // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
 void Initialize_DMP(void){
         I2Cdev::initializeI2C();
         IntMasterEnable();
         // initialize device
         mpu.initialize();
         devStatus = mpu.dmpInitialize();
         // supply your own gyro offsets here, scaled for min sensitivity
         mpu.setXGyroOffset(220);
         mpu.setYGyroOffset(76);
         mpu.setZGyroOffset(-85);
         mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
         if (devStatus == 0)
         {

             mpu.setDMPEnabled(true);
             mpuIntStatus = mpu.getIntStatus();
             INTA_SETUP();
             dmpReady = true;
             packetSize = mpu.dmpGetFIFOPacketSize();
         }
         else
         {
             //return 0;
         }

 }
  void GetYawPitchRoll(void){
      // if programming failed, don't try to do anything
             if (!dmpReady)
                 while(dmpReady);//return 0;

             // wait for MPU interrupt or extra packet(s) available
             while (!mpuInterrupt && fifoCount < packetSize)
             {
                 INTA_TOGGLE();
             }

             // reset interrupt flag and get INT_STATUS byte
             mpuInterrupt = false;
             mpuIntStatus = mpu.getIntStatus();

             // get current FIFO count
             fifoCount = mpu.getFIFOCount();

             // check for overflow (this should never happen unless our code is too inefficient)
             if ((mpuIntStatus & 0x10) || fifoCount == 1024)
             {
                 // reset so we can continue cleanly
                 mpu.resetFIFO();

                 // otherwise, check for DMP data ready interrupt (this should happen frequently)
             }
             else if (mpuIntStatus & 0x02)
             {
                 // wait for correct available data length, should be a VERY short wait
                 while (fifoCount < packetSize)
                     fifoCount = mpu.getFIFOCount();

                 // read a packet from FIFO
                 mpu.getFIFOBytes(fifoBuffer, packetSize);

                 // track FIFO count here in case there is > 1 packet available
                 // (this lets us immediately read more without waiting for an interrupt)
                 fifoCount -= packetSize;

                 // display Euler angles in degrees
                 mpu.dmpGetQuaternion(&q, fifoBuffer);
                 mpu.dmpGetGravity(&gravity, &q);
                 mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
             }
  }


