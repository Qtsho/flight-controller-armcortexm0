/**
 * main.c
 */
#include "include.h"
#include "DMPTESTING/MPU6050_6Axis_MotionApps20.h"
#include "utils/uartstdio.h"
#include "utils/uartstdio.c"

#define __DMP__
#ifdef __DMP__
void InitConsole(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 9600, 16000000);
}
void main()
{
    SysCtlClockSet(
            SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    float yaw,pitch,roll;
    InitConsole();
    Config_SysTick();
    I2Cdev::initializeI2C(); //I2C initialize
    //IntMasterEnable();//Interrupt enable
    Initialize_DMP();//Initilaize DMP and MPU
    UARTprintf("Starting reading now ...");
    SysCtlDelay(SysCtlClockGet()/3);
    while(1)
    {
        GetYawPitchRoll();
        yaw=(ypr[0]*180)/3.14;
        pitch=(ypr[1]*180)/3.14;
        roll=(ypr[2]*180)/3.14;
        //..........
        char syaw[20];
        char spitch[20];
        char sroll[20];
        ftoa(abs(yaw), syaw, 4);
        ftoa(abs(roll), sroll, 4);
        ftoa(abs(pitch), spitch, 4);
        if(yaw<0)
        UARTprintf("Yaw: -%s ",syaw);
        else UARTprintf("Yaw: %s ",syaw);
        if(pitch<0)
        UARTprintf("pitch: -%s ",spitch);
        else UARTprintf("pitch: %s ",spitch);
        if(roll<0)
        UARTprintf("Roll: -%s ",sroll);
        else UARTprintf("Roll: %s ",sroll);
        UARTprintf("FIFOCount: %d \n",fifoCount);
        //...........
        SysCtlDelay(SysCtlClockGet()/50);
    }
}
#endif
#ifdef Calibration
int buffersize = 1000; //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8;//Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;//Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x699*+
//MPU6050 accelgyro;
MPU6050 accelgyro(0x68);// <-- use for AD0 high

int16_t ax, ay, az, gx, gy, gz;

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, stateX = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
void InitConsole(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 9600, 16000000);
}
void meansensors(void)
{
    long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0,
    buff_gz = 0;

    while (i < (buffersize + 101))
    {
        // read raw accel/gyro measurements from device
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        if (i > 100 && i <= (buffersize + 100))
        { //First 100 measures are discarded
            buff_ax = buff_ax + ax;
            buff_ay = buff_ay + ay;
            buff_az = buff_az + az;
            buff_gx = buff_gx + gx;
            buff_gy = buff_gy + gy;
            buff_gz = buff_gz + gz;
        }
        if (i == (buffersize + 100))
        {
            mean_ax = buff_ax / buffersize;
            mean_ay = buff_ay / buffersize;
            mean_az = buff_az / buffersize;
            mean_gx = buff_gx / buffersize;
            mean_gy = buff_gy / buffersize;
            mean_gz = buff_gz / buffersize;
        }
        i++;
        SysCtlDelay((SysCtlClockGet() / 1000) * 2); //Needed so we don't get repeated measures
    }
}
void calibration(void)
{
    ax_offset = -mean_ax / 8;
    ay_offset = -mean_ay / 8;
    az_offset = (16384 - mean_az) / 8;

    gx_offset = -mean_gx / 4;
    gy_offset = -mean_gy / 4;
    gz_offset = -mean_gz / 4;
    while (1)
    {
        int ready = 0;
        accelgyro.setXAccelOffset(ax_offset);
        accelgyro.setYAccelOffset(ay_offset);
        accelgyro.setZAccelOffset(az_offset);

        accelgyro.setXGyroOffset(gx_offset);
        accelgyro.setYGyroOffset(gy_offset);
        accelgyro.setZGyroOffset(gz_offset);

        meansensors();

        if (abs(mean_ax) <= acel_deadzone)
        ready++;
        else
        ax_offset = ax_offset - mean_ax / acel_deadzone;

        if (abs(mean_ay) <= acel_deadzone)
        ready++;
        else
        ay_offset = ay_offset - mean_ay / acel_deadzone;

        if (abs(16384 - mean_az) <= acel_deadzone)
        ready++;
        else
        az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

        if (abs(mean_gx) <= giro_deadzone)
        ready++;
        else
        gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

        if (abs(mean_gy) <= giro_deadzone)
        ready++;
        else
        gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

        if (abs(mean_gz) <= giro_deadzone)
        ready++;
        else
        gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

        if (ready == 6)
        break;
    }
}

void main(void)
{
    SysCtlClockSet(
            SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
//          Config_SysTick();
    I2Cdev::initializeI2C();//I2C initialize
    // join I2C bus (I2Cdev library doesn't do this automatically)
    //    I2Cdev::initializeI2C();
    // initialize device
    InitConsole();
    accelgyro.initialize();
    // reset offsets
    accelgyro.setXAccelOffset(0);
    accelgyro.setYAccelOffset(0);
    accelgyro.setZAccelOffset(0);
    accelgyro.setXGyroOffset(0);
    accelgyro.setYGyroOffset(0);
    accelgyro.setZGyroOffset(0);
    //..............................
    if (stateX == 0)
    {
        UARTprintf("\nReading sensors for first time..."); // Serial.println("\nReading sensors for first time...");
        meansensors();
        stateX++;
        SysCtlDelay((SysCtlClockGet()/1000));
    }

    if (stateX == 1)
    {
        UARTprintf("\nCalculating offsets..."); //Serial.println("\nCalculating offsets...");
        calibration();
        stateX++;
        SysCtlDelay((SysCtlClockGet()/1000));
    }

    if (stateX == 2)
    {
        meansensors();
        UARTprintf("\nAccX,AccY,AccZ,GyX,GyY,GyZ");
        UARTprintf("\n%d %d %d %d %d %d", ax_offset, ay_offset, az_offset,
                gx_offset, gy_offset, gz_offset);
        UARTprintf("\nFINISHED!");            // Serial.println("\nFINISHED!");
        UARTprintf("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
        UARTprintf(
                "Check that your sensor readings are close to 0 0 16384 0 0 0");
        UARTprintf("\n%d %d %d %d %d %d", mean_ax, mean_ay, mean_az, mean_gx,
                mean_gy, mean_gz);
        UARTprintf(
                "\nIf calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
        // Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
        //  Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
        // Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
        while (1)
        {
            Config_BLUE();
        };
    }

}

#endif
#ifdef __YPR__
void main(void)
{
    float offy,offp,offr;
    SysCtlClockSet(
            SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    I2Cdev::initializeI2C(); //I2C initialize
    IntMasterEnable();//Interrupt enable
    Initialize_DMP();
    int8_t i=0;
    while(i<1000)
    {
        GetYawPitchRoll();
        offy+=ypr[0];
        offp+=ypr[1];
        offr+=ypr[2];
        i++;
    }
    offy/=1000;
    offp/=1000;
    offr/=1000;
    while(1)
    {
        GetYawPitchRoll();
        float yaw = ((ypr[0]-offy) * 180) / 3.14;
        float pitch = ((ypr[1]-offp) * 180) / 3.14;
        float roll=((ypr[2]-offr)*180)/3.14;
    }
}
#endif
#ifdef TESTING

void InitConsole(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, 16000000);
}

void main(void)
{
    float yaw, pitch, roll;
    SysCtlClockSet(
    SYSCTL_SYSDIV_12_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    MPU6050 mpu;
    uint16_t packetSize;
    uint16_t fifoCount;
    uint8_t fifoBuffer[64];
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];

    I2Cdev::initializeI2C();
    InitConsole();
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setXAccelOffset(-1343);
    mpu.setYAccelOffset(-1155);
    mpu.setZAccelOffset(1033);
    mpu.setXGyroOffset(19);
    mpu.setYGyroOffset(-27);
    mpu.setZGyroOffset(16);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    fifoCount = mpu.getFIFOCount();
    UARTprintf("Starting reading now ...");
    SysCtlDelay(SysCtlClockGet()/3);
    while (1)
    {
        while (fifoCount < packetSize)
        {
            fifoCount = mpu.getFIFOCount();
        }

        if (fifoCount == 1024)
        {
            mpu.resetFIFO();
        }
        else
        {
            if (fifoCount % packetSize != 0)
            {
                mpu.resetFIFO();
            }
            else
            {

                while (fifoCount >= packetSize)
                {

                    mpu.getFIFOBytes(fifoBuffer, packetSize);
                    fifoCount -= packetSize;

                }
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            }

        }
        yaw = (ypr[0] * 180) / 3.14;
        pitch = (ypr[1] * 180) / 3.14;
        roll = (ypr[2] * 180) / 3.14;
        //..........
        char syaw[20];
        char spitch[20];
        char sroll[20];
        ftoa(abs(yaw), syaw, 4);
        ftoa(abs(roll), sroll, 4);
        ftoa(abs(pitch), spitch, 4);
        if (yaw < 0)
            UARTprintf("Yaw: -%s ", syaw);
        else
            UARTprintf("Yaw: %s ", syaw);
        if (pitch < 0)
            UARTprintf("pitch: -%s ", spitch);
        else
            UARTprintf("pitch: %s ", spitch);
        if (roll < 0)
            UARTprintf("Roll: -%s \n", sroll);
        else
            UARTprintf("Roll: %s \n", sroll);
        //...........
        SysCtlDelay(SysCtlClockGet() / 50);
    }

}

#endif
