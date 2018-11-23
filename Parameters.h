/*
 * Parameters.h
 *
 *  Created on: Aug 15, 2017
 *      Author: Quang Tien
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

float pid_p_gain_roll = 3.0;   //2.8      //5.4 over compensate      //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.06;   //0.0           //Gain setting for the roll I-controller
float pid_d_gain_roll = 22.0;  //22           //Gain setting for the roll D-controller
int pid_max_roll = 400;

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 7.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.01;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)


float ypr_temp[3];

int16_t throttle,yaw,pitch,roll,button;
int16_t input_yaw_signal, input_roll_signal, input_pitch_signal,throttle_signal;

int temperature;
float roll_level_adjust, pitch_level_adjust;

double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll,angle_yaw,acc_total_vector;
int16_t ax, ay, az, gx, gy, gz;
double gyro_calibX,gyro_calibY,gyro_calibZ;
int16_t GyX, GyY, GyZ;
//for calib...


#define MAX_THROTTLE 2000
#define MIN_THROTTLE 1000
#define ARMING_THROTTLE 1030
bool send = false;

#endif /* PARAMETERS_H_ */
