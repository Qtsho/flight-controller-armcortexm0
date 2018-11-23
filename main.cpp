/**
 * main.c
 */

#include "include.h"
#include "DMP/MPU6050_6Axis_MotionApps20.h"
#include "Parameters.h"
static uint8_t i = 0;
void calculate_pid(void);
#define CALIBRATEIMU
//#define TUNNING
//#define DEBUG
#define ADC
//#define TESTPWM
float a = 0;
uint8_t errors_count;
int main(void)
{
    SysCtlClockSet(
    SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //40MHz
#ifdef DEBUG
            SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
            GPIOPinConfigure(GPIO_PA0_U0RX);
            GPIOPinConfigure(GPIO_PA1_U0TX);
            SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
            UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
            GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
            UARTStdioConfig(0, 9600, 16000000);

#endif
    // Config_ADC();
    // while(Voltage < 2.0);
    Config_BLUE();
    MPU6050 mpu;
    IntMasterEnable();
    Config_Timer();

#ifdef ADC
    Config_ADC();
#endif
    Config_PWM();
    ConfigDMA();
    InitUART1Transfer();
    I2Cdev::initializeI2C();
#ifdef _DMP_
    Initialize_DMP();
#else
    //  IntMasterEnable();
    // initialize device
    NoneDMPInitialize();
    // Set By Pass Mode for using Compass
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(278);
    mpu.setYGyroOffset(60);
    mpu.setZGyroOffset(8);
    mpu.setXAccelOffset(693);
    mpu.setYAccelOffset(-1231);
    mpu.setZAccelOffset(1358); // 1688 factory default for my test chip
#endif

//#define CALIB_ESC
#ifdef CALIB_ESC
//*******************calbrate ESC**************************
    esc1Write(MAX_THROTTLE);
    SysCtlDelay(2);
    esc2Write(MAX_THROTTLE);
    SysCtlDelay(2);
    esc3Write(MAX_THROTTLE);
    SysCtlDelay(2);
    esc4Write(MAX_THROTTLE);
    delayMS(3000);
    esc1Write(MIN_THROTTLE);
    SysCtlDelay(2);// PD0
    esc2Write(MIN_THROTTLE);
    SysCtlDelay(2);// PD1
    esc3Write(MIN_THROTTLE);
    SysCtlDelay(2);// PE4
    esc4Write(MIN_THROTTLE);// PE5
    delayMS(2000);
#endif
#ifndef CALIB_ESC
    esc1Write(MIN_THROTTLE);
    SysCtlDelay(2); // PD0
    esc2Write(MIN_THROTTLE);
    SysCtlDelay(2); // PD1
    esc3Write(MIN_THROTTLE);
    SysCtlDelay(2); // PE4
    esc4Write(MIN_THROTTLE); // PE5
    delayMS(2000);
    delayMS(2000);
#endif
//************************** Calibrate Angle........//
#ifdef CALIBRATEIMU

    ypr_temp[1] = 0;
    ypr_temp[2] = 0;
    gyro_calibX = 0;
    gyro_calibY = 0;
    gyro_calibZ = 0;
    int q = 0;
    for (q = 0; q < 2000; q++)
    {
        gyro_calibX += mpu.getRotationX();
        SysCtlDelay(3);
        gyro_calibY += mpu.getRotationY();
        SysCtlDelay(3);
        gyro_calibZ += mpu.getRotationZ();
        SysCtlDelay(3);
#ifdef _DMP_
        GetYawPitchRoll();
        ypr_temp[1]+= ypr[1];
        ypr_temp[2]+= ypr[2];
#else
        ax = mpu.getAccelerationX();
        ay = mpu.getAccelerationY();
        az = mpu.getAccelerationZ();
        acc_total_vector = sqrt((ax * ax) + (ay * ay) + (az * az)); //pytago
        angle_pitch_acc = asin((float) ay / acc_total_vector) * 57.296;//Calculate the pitch angle
        angle_roll_acc = asin((float) ax / acc_total_vector) * -57.296;// 57.29= 1/(pi*180)
        ypr_temp[1] += angle_pitch_acc;
        ypr_temp[2] += angle_roll_acc;
#endif
        //****** Blinky LED while IMU CALIBRATING******//////////////
        if ((q % 50) == 0)
        {
            if (i == 0)
            i = GPIO_PIN_0;
            else
            i = 0;
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, i);

        }

    }  //***** END CALIBRATION****//
    gyro_calibX /= 2000;
    gyro_calibY /= 2000;
    gyro_calibZ /= 2000;
    ypr_temp[1] /= 2000;
    ypr_temp[2] /= 2000;
#endif
#ifndef CALIBRATEIMU
    gyro_calibX = -40.73199;
    gyro_calibY = 124.672195;
    gyro_calibZ = -81.65324;
    ypr_temp[1] = -1.718912425;
    ypr_temp[2] = -2.752992035;
#endif
//...........................................................//
    TimerEnable(TIMER0_BASE, TIMER_A);
    flag = 0;
    start = 0;

    //MAIN LOOP HERE.............................................
    //..........................................................
    //..........................................................
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0); //LED 2
    while (1)
    {

        if (1 == flag)
        {

            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            if (ax == 0 && ay == 0 && az == 0 && gx == 0 && gy == 0 && gz == 0)
            {
                GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0); //LED 2
            }

            if (send)
            {
                esc1Write(esc_1);
                esc2Write(esc_2);
                esc3Write(esc_3);
                esc4Write(esc_4);
                send = false;
            }
            //..........DEBUG Toggle PF3....................

            if (i == 0)
                i = GPIO_PIN_2;
            else
                i = 0;
            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, i);

            //....Rotation Input for PID....................

#ifdef _DMP_
            gx = gyro[0];
            gy = gyro[1];
            gz = gyro[2];
//            gx -= gyro_calibX;
//            gy -= gyro_calibY;
//            gz -= gyro_calibZ;
            float params = 1.0;
#else

            gx -= gyro_calibX;
            gy -= gyro_calibY;
            gz -= gyro_calibZ;
            gyro_pitch = gx;
            gyro_roll = gy;
            gyro_yaw = gz * (-1);

#ifndef DEBUG
            ypr[1] += gyro_pitch * 0.000061068; //0.000061068;// /65.5 *4ms 000061068 ( 0.000076335=5ms)
            ypr[2] += gyro_roll * 0.000061068;
#endif
#ifdef DEBUG
            ypr[1] += gyro_pitch*0.0015267; // /65.5 *4ms 000061068 ( 0.000076335=5ms)
            ypr[2] += gyro_roll *0.0015267;
#endif

            ypr[1] -= ypr[2] * sin(gyro_yaw * 0.000001066);
            ypr[2] += ypr[1] * sin(gyro_yaw * 0.000001066);
            //**************************
            acc_total_vector = sqrt((ax * ax) + (ay * ay) + (az * az)); //pytago
            if (abs(ay) < acc_total_vector)
            {                       //Prevent the asin function to produce a NaN
                angle_pitch_acc = asin((float) ay / acc_total_vector) * 57.296; //Calculate the pitch angle.
            }
            if (abs(ax) < acc_total_vector)
            {                       //Prevent the asin function to produce a NaN
                angle_roll_acc = asin((float) ax / acc_total_vector) * -57.296; //Calculate the roll angle.
            }
//            angle_pitch_acc = asin((float) ay / acc_total_vector) * 57.296; //Calculate the pitch angle
//            angle_roll_acc = asin((float) ax / acc_total_vector) * -57.296; // 57.29= 1/(pi*180)
            //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
            angle_pitch_acc -= ypr_temp[1]; //Accelerometer calibration value for pitch.
            angle_roll_acc -= ypr_temp[2];
            //**********************
            float comp_f = 0.9996;
            ypr[1] = ypr[1] * comp_f + angle_pitch_acc * (1 - comp_f);
            ypr[2] = ypr[2] * comp_f + angle_roll_acc * (1 - comp_f);

// for PID INPUT
            gyro_roll_input = (gyro_roll_input * 0.7)
                    + ((gyro_roll / 65.5) * 0.3); //Gyro pid input is deg/sec.
            gyro_pitch_input = (gyro_pitch_input * 0.7)
                    + ((gyro_pitch / 65.5) * 0.3); //Gyro pid input is deg/sec.
            gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3); //Gyro pid input is deg/sec.

#endif

            //...Get DMP for angle........................
#ifdef _DMP_
            GetYawPitchRoll();
            angle_roll = ((ypr[1] - ypr_temp[1]) * 180) / 3.14;
            angle_pitch = ((ypr[2] - ypr_temp[2]) * 180) / 3.14;
#else
            angle_roll = ypr[2]; // doi goc lai
            angle_pitch = ypr[1];
#endif
#ifdef DEBUG
            char sroll[20],spitch[20];
            if (angle_roll < 0)
            {
                angle_roll = abs(angle_roll);
                ftoa(angle_roll, sroll, 4);
                UARTprintf("roll -%s \t  ", sroll);
            }
            else
            {
                ftoa(angle_roll, sroll, 4);
                UARTprintf("roll %s \t  ", sroll);
            }
            if (angle_pitch < 0)
            {
                angle_pitch = abs(angle_pitch);
                ftoa(angle_pitch, spitch, 4);
                UARTprintf("pitch- %s \n ", spitch);
            }
            else
            {
                ftoa(angle_pitch, spitch, 4);
                UARTprintf("pitch %s \n ", spitch);
            }

#else
            pitch_level_adjust = angle_pitch * 15;
            roll_level_adjust = angle_roll * 15;
            //..************************** GRAB the signal here ******************************//

            yaw = input_yaw;
            pitch = input_pitch;
            roll = input_roll;
            throttle = input_throttle; // get throttle signal
            button = input_button;

            pid_pitch_setpoint = 0;
            pid_roll_setpoint = 0;
            pid_yaw_setpoint = 0;

            //**************** SWITCHING HANDLE ************************

            if (correct)
            {
                a = 0; // How much do signal depend on the past?
                errors_count = 0;
                //**************** PROCEDURE PUT HHERE *********************************//
#ifdef TUNNING

                if (button ==1 )
                {
                    start = 2;

                }
                else
                {
                    start = 0;
                }
#endif
#ifndef TUNNING
                if (yaw < 1050 && throttle < 1050 && lost == false) // arming session RED LED
                {

                    start = 1;
                    pid_i_mem_roll = 0;
                    pid_last_roll_d_error = 0;
                    pid_i_mem_pitch = 0;
                    pid_last_pitch_d_error = 0;
                    pid_i_mem_yaw = 0;
                    pid_last_yaw_d_error = 0;

                    //reset for not bumpless start
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1); // Red is ready to fly
                }
                if (start == 1 && throttle < 1050 && yaw > 1450) // flying session GREEN LED
                {
                    start = 2; // bat dau  dem 3s
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); // green

                }
                if (start == 2 && throttle < 1050)
                    begintimer = true;
                else
                    begintimer = false;

                if (start == 2 && throttle < 1050 && yaw > 1950
                        && lost == false) // disarn
                {
                    start = 0;
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
                }
                if (timeout && throttle < ARMING_THROTTLE && start == 2
                        && lost == false) // timeout save batterry
                {
                    start = 0;
                    timeout = false;
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
                }
#endif

                //*********************************************************

            }
            else
            {
                a = 1;
                errors_count++;
            }
            if (errors_count < 5)
            {
                input_roll_signal = a * input_roll_signal + (1 - a) * roll;
                input_pitch_signal = a * input_pitch_signal + (1 - a) * pitch;
                input_yaw_signal = a * input_yaw_signal + (1 - a) * yaw;
                throttle_signal = a * throttle_signal + (1 - a) * throttle;

            }
            //TINH HIEU FAIL
            else
            {
                input_roll_signal = 0.8 * input_roll_signal + 0.2 * 1500;
                input_pitch_signal = 0.8 * input_pitch + 0.2 * 1500;
                input_yaw_signal = 0.8 * input_yaw_signal + 0.2 * 1500;
                throttle_signal = throttle;
            }
            // MAT TINH HIEU
            if (lost)
            {
                input_roll_signal = 0.8 * input_roll_signal + 0.2 * 1500;
                input_pitch_signal = 0.8 * input_pitch + 0.2 * 1500;
                input_yaw_signal = 0.8 * input_yaw_signal + 0.2 * 1500;
                throttle_signal = throttle;
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
                GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
            }
            else
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
                GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
            }
            //************************************************************
#ifndef TUNNING
            if (input_pitch_signal > 1508 && input_pitch_signal < 2100)
                pid_pitch_setpoint = input_pitch_signal - 1520;
            else if (input_pitch_signal < 1492 && input_pitch_signal > 990)
                pid_pitch_setpoint = input_pitch_signal - 1480;
            if (input_roll_signal > 1508 && input_roll_signal < 2100)
                pid_roll_setpoint = input_roll_signal - 1520;
            else if (input_roll_signal < 1492 && input_roll_signal > 990)
                pid_roll_setpoint = input_roll_signal - 1480;
#endif
#ifdef YAW_EN
            if (throttle_signal > 1050)
            { //Do not yaw when turning off the motors.
                if (input_yaw_signal > 1508)
                pid_yaw_setpoint = (input_yaw_signal - 1508) / 3.0;
                else if (input_yaw_signal < 1492)
                pid_yaw_setpoint = (input_yaw_signal - 1492) / 3.0;
            }
#endif
            pid_pitch_setpoint -= pitch_level_adjust;
            pid_roll_setpoint -= roll_level_adjust;
            pid_pitch_setpoint /= 3.0;
            pid_roll_setpoint /= 3.0;

            calculate_pid();
            //...................................

            if (start == 2)
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

                if (throttle_signal > 1800)
                    throttle_signal = 1800; //We need some room to keep full control at full throttle.

                esc_1 = throttle_signal - pid_output_pitch + pid_output_roll
                        - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
                esc_2 = throttle_signal + pid_output_pitch + pid_output_roll
                        + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
                esc_3 = throttle_signal + pid_output_pitch - pid_output_roll
                        - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
                esc_4 =
                        throttle_signal - pid_output_pitch - pid_output_roll
                                + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
#ifdef ADC
                                if (Voltage < 8.4 && Voltage > 6.4)
                                {                   //Is the battery connected?b
                                    esc_1 += esc_1 * ((840.0 - Voltage * 100) / (float) 3500);//Compensate the esc-1 pulse for voltage drop.
                                    esc_2 += esc_2 * ((840.0 - Voltage * 100) / (float) 3500);//Compensate the esc-2 pulse for voltage drop.
                                    esc_3 += esc_3 * ((840.0 - Voltage * 100) / (float) 3500);//Compensate the esc-3 pulse for voltage drop.
                                    esc_4 += esc_4 * ((840.0 - Voltage * 100) / (float) 3500);//Compensate the esc-4 pulse for voltage drop.
                                }
#endif
                if (esc_1 < 1050)
                    esc_1 = 1050;                     //Keep the motors running.
                if (esc_2 < 1050)
                    esc_2 = 1050;                     //Keep the motors running.
                if (esc_3 < 1050)
                    esc_3 = 1050;                     //Keep the motors running.
                if (esc_4 < 1050)
                    esc_4 = 1050;                     //Keep the motors running.

                if (esc_1 > MAX_THROTTLE)
                    esc_1 = MAX_THROTTLE;     //Limit the esc-1 pulse to 2000us.
                if (esc_2 > MAX_THROTTLE)
                    esc_2 = MAX_THROTTLE;     //Limit the esc-2 pulse to 2000us.
                if (esc_3 > MAX_THROTTLE)
                    esc_3 = MAX_THROTTLE;     //Limit the esc-3 pulse to 2000us.
                if (esc_4 > MAX_THROTTLE)
                    esc_4 = MAX_THROTTLE;     //Limit the esc-4 pulse to 2000us.
                send = true;
            }
//            else if (start == 1)
//            {
//                esc_1 = ARMING_THROTTLE; //If start is not 2 keep a 1000us pulse for ess-1.
//                esc_2 = ARMING_THROTTLE; //If start is not 2 keep a 1000us pulse for ess-2.
//                esc_3 = ARMING_THROTTLE; //If start is not 2 keep a 1000us pulse for ess-3.
//                esc_4 = ARMING_THROTTLE; //If start is not 2 keep a 1000us pulse for ess-4.
//                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
//
//                send = true;
//            }
            else if (start == 0)
            {
                esc_1 = MIN_THROTTLE; //If start is not 2 keep a 1000us pulse for ess-1.
                esc_2 = MIN_THROTTLE; //If start is not 2 keep a 1000us pulse for ess-2.
                esc_3 = MIN_THROTTLE; //If start is not 2 keep a 1000us pulse for ess-3.
                esc_4 = MIN_THROTTLE; //If start is not 2 keep a 1000us pulse for ess-4.
                send = true;
            }
#endif

            flag = 0;
        } // end if check flag
    } // end while 1
} // end main
void calculate_pid(void)
{
    //Roll calculations
    pid_error_temp = gyro_roll_input - pid_roll_setpoint;
    pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
    if (pid_i_mem_roll > pid_max_roll)
        pid_i_mem_roll = pid_max_roll;
    else if (pid_i_mem_roll < pid_max_roll * -1)
        pid_i_mem_roll = pid_max_roll * -1;

    pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll
            + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
    if (pid_output_roll > pid_max_roll)
        pid_output_roll = pid_max_roll;
    else if (pid_output_roll < pid_max_roll * -1)
        pid_output_roll = pid_max_roll * -1;

    pid_last_roll_d_error = pid_error_temp;

    //Pitch calculations
    pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
    pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
    if (pid_i_mem_pitch > pid_max_pitch)
        pid_i_mem_pitch = pid_max_pitch;
    else if (pid_i_mem_pitch < pid_max_pitch * -1)
        pid_i_mem_pitch = pid_max_pitch * -1;

    pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch
            + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
    if (pid_output_pitch > pid_max_pitch)
        pid_output_pitch = pid_max_pitch;
    else if (pid_output_pitch < pid_max_pitch * -1)
        pid_output_pitch = pid_max_pitch * -1;

    pid_last_pitch_d_error = pid_error_temp;

    //Yaw calculations
    pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
    pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
    if (pid_i_mem_yaw > pid_max_yaw)
        pid_i_mem_yaw = pid_max_yaw;
    else if (pid_i_mem_yaw < pid_max_yaw * -1)
        pid_i_mem_yaw = pid_max_yaw * -1;

    pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw
            + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
    if (pid_output_yaw > pid_max_yaw)
        pid_output_yaw = pid_max_yaw;
    else if (pid_output_yaw < pid_max_yaw * -1)
        pid_output_yaw = pid_max_yaw * -1;

    pid_last_yaw_d_error = pid_error_temp;

}
