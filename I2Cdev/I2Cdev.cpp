// I2Cdev library collection - Main I2C device class
// Abstracts bit and byte I2C R/W functions into a convenient class
// 2013-06-05 by Jeff Rowberg <jeff@rowberg.net>
//
// Changelog:
//      2013-05-06 - add Francesco Ferrara's Fastwire v0.24 implementation with small modifications
//      2013-05-05 - fix issue with writing bit values to words (Sasquatch/Farzanegan)
//      2012-06-09 - fix major issue with reading > 32 bytes at a time with Arduino Wire
//                 - add compiler warnings when using outdated or IDE or limited I2Cdev implementation
//      2011-11-01 - fix write*Bits mask calculation (thanks sasquatch @ Arduino forums)
//      2011-10-03 - added automatic Arduino version detection for ease of use
//      2011-10-02 - added Gene Knight's NBWire TwoWire class implementation with small modifications
//      2011-08-31 - added support for Arduino 1.0 Wire library (methods are different from 0.x)
//      2011-08-03 - added optional timeout parameter to read* methods to easily change from default
//      2011-08-02 - added support for 16-bit registers
//                 - fixed incorrect Doxygen comments on some methods
//                 - added timeout value for read operations (thanks mem @ Arduino forums)
//      2011-07-30 - changed read/write function structures to return success or byte counts
//                 - made all methods static for multi-device memory savings
//      2011-07-28 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2013 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "I2Cdev.h"


/** Default constructor.
 */
uint8_t min(int8_t num1,int8_t num2){
    if (num1<num2) return num1;
    else return num2;
}
I2Cdev::I2Cdev() {
}

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout) {
    uint8_t b;
    uint8_t count = readByte(devAddr, regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return count;
}

/** Read a single bit from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout) {
    uint16_t b;
    uint8_t count = readWord(devAddr, regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return count;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = readByte(devAddr, regAddr, &b, timeout)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

/** Read multiple bits from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
 */
int8_t I2Cdev::readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout) {
    // 1101011001101001 read byte
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    //    010           masked
    //           -> 010 shifted
    uint8_t count;
    uint16_t w;
    if ((count = readWord(devAddr, regAddr, &w, timeout)) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        w &= mask;
        w >>= (bitStart - length + 1);
        *data = w;
    }
    return count;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout) {
    return readBytes(devAddr, regAddr, 1, data, timeout);
}

/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout) {
    return readWords(devAddr, regAddr, 1, data, timeout);
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Number of bytes read (-1 indicates failure)
 */
int8_t I2Cdev::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout) {
    int8_t count = 0;
    unsigned long t1 = SysCount;
            for (uint8_t k = 0; k < length; k += min(length, BUFFER_LENGTH)) {
               uint8_t buffer=(uint8_t)min(length - k, BUFFER_LENGTH); //Wire.requestFrom(devAddr, (uint8_t)min(length - k, BUFFER_LENGTH));
                //........................................
                I2CMasterSlaveAddrSet(I2C1_BASE, devAddr, false); //Wire.beginTransmission(devAddr);
                I2CMasterDataPut(I2C1_BASE, regAddr);            //Wire.send(regAddr);
                I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND); //Wire.endTransmission();
                while (I2CMasterBusy(I2C1_BASE));
                I2CMasterSlaveAddrSet(I2C1_BASE, devAddr, true);
                I2CMasterControl(I2C1_BASE,I2C_MASTER_CMD_BURST_RECEIVE_START);
                while (I2CMasterBusy(I2C1_BASE));
                data[0]=I2CMasterDataGet(I2C1_BASE);

                //....................................................
                for (count=1; count<(buffer-1) &&(timeout == 0 || SysCount - t1 < timeout); count++) {
                    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
                    while (I2CMasterBusy(I2C1_BASE));
                    data[count] = I2CMasterDataGet(I2C1_BASE);

                }
                I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
                while (I2CMasterBusy(I2C1_BASE));
                data[count] = I2CMasterDataGet(I2C1_BASE);

               // Wire.endTransmission();
            }
    // check for timeout
    if (timeout > 0 && SysCount - t1 >= timeout && count < length) count = -1; // timeout
    return count;
}

/** Read multiple words from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Number of words read (-1 indicates failure)
 */
int8_t I2Cdev::readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout) {
    int8_t count = 0;
    unsigned long t1 = SysCount;
      for (uint8_t k = 0; k < length * 2; k += min(length * 2, BUFFER_LENGTH)) {
                I2CMasterSlaveAddrSet(I2C1_BASE, devAddr, false); //Wire.beginTransmission(devAddr);
                I2CMasterDataPut(I2C1_BASE, regAddr);            //Wire.send(regAddr);
                I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND); //Wire.endTransmission();
                while (I2CMasterBusy(I2C1_BASE));
                //.....................................
                I2CMasterSlaveAddrSet(I2C1_BASE, devAddr, true);
                I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
                while (I2CMasterBusy(I2C1_BASE));
                data[0]=I2CMasterDataGet(I2C1_BASE)<<8;
                //.........................
                bool msb = true; // starts with MSB, then LSB
                for (;count < length && (timeout == 0 || SysCount - t1 < timeout);) {
                    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
                    while (I2CMasterBusy(I2C1_BASE));
                    if (msb) {
                        // first byte is bits 15-8 (MSb=15)
                        data[count] = I2CMasterDataGet(I2C1_BASE) << 8;
                    } else {
                        // second byte is bits 7-0 (LSb=0)
                        data[count] |= I2CMasterDataGet(I2C1_BASE);
                        count++;
                    }
                    msb = !msb;
                }
                I2CMasterControl(I2C1_BASE,I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
                while (I2CMasterBusy(I2C1_BASE));
                data[count] |= I2CMasterDataGet(I2C1_BASE);
            }


    if (timeout > 0 && SysCount - t1 >= timeout && count < length) count = -1; // timeout

    return count;
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    readByte(devAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(devAddr, regAddr, b);
}

/** write a single bit in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data) {
    uint16_t w;
    readWord(devAddr, regAddr, &w);
    w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
    return writeWord(devAddr, regAddr, w);
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (readByte(devAddr, regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(devAddr, regAddr, b);
    } else {
        return false;
    }
}

/** Write multiple bits in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data) {
    //              010 value to write
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    // 0001110000000000 mask word
    // 1010111110010110 original value (sample)
    // 1010001110010110 original & ~mask
    // 1010101110010110 masked | value
    uint16_t w;
    if (readWord(devAddr, regAddr, &w) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        w &= ~(mask); // zero all important bits in existing word
        w |= data; // combine data with existing word
        return writeWord(devAddr, regAddr, w);
    } else {
        return false;
    }
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    return writeBytes(devAddr, regAddr, 1, &data);
}

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data) {
    return writeWords(devAddr, regAddr, 1, &data);
}

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) {

        I2CMasterSlaveAddrSet(I2C1_BASE, devAddr, false); // Set to write mode
        I2CMasterDataPut(I2C1_BASE, regAddr); // Place address into data register
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START); // Send start condition
        while (I2CMasterBusy(I2C1_BASE))
            ; // Wait until transfer is done
        uint8_t i = 0;
        for (i = 0; i < length - 1; i++)
        {
            I2CMasterDataPut(I2C1_BASE, data[i]); // Place data into data register
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_CONT); // Send continues condition
            while (I2CMasterBusy(I2C1_BASE))
                ; // Wait until transfer is done
        }

        I2CMasterDataPut(I2C1_BASE, data[length - 1]); // Place data into data register
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH); // Send finish condition
        while (I2CMasterBusy(I2C1_BASE))
            ; // Wait until transfer is done
    return true;
}

/** Write multiple words to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data) {

            I2CMasterSlaveAddrSet(I2C1_BASE, devAddr, false); //Wire.beginTransmission(devAddr);
            I2CMasterDataPut(I2C1_BASE, regAddr);            //Wire.send(regAddr);
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
            while(I2CMasterBusy(I2C1_BASE));
            uint8_t i = 0;
    for (i = 0; i < 2*length -1; i++) {

           I2CMasterDataPut(I2C1_BASE, (uint8_t)(data[i]>>8));// Wire.send((uint8_t)(data[i] >> 8));     // send MSB
           I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
           while(I2CMasterBusy(I2C1_BASE));
           I2CMasterDataPut(I2C1_BASE, (uint8_t)data[i++]);// Wire.send((uint8_t)data[i++]);          // send LSB
           I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
           while(I2CMasterBusy(I2C1_BASE));
    }
    I2CMasterDataPut(I2C1_BASE, (uint8_t)data[2*length-1]);
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C1_BASE));

    return true;
}
void I2Cdev::initializeI2C(){
      SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1); // Enable I2C1 peripheral
      SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable GPIOA peripheral
      SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated

      // Use alternate function
      GPIOPinConfigure(GPIO_PA6_I2C1SCL);
      GPIOPinConfigure(GPIO_PA7_I2C1SDA);

      GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6); // Use pin with I2C SCL peripheral
      GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7); // Use pin with I2C peripheral

      I2CMasterInitExpClk(I2C1_BASE, (SysCtlClockGet()*2)/5, true); // Enable and set frequency to 400 kHz

      SysCtlDelay(2); // Insert a few cycles after enabling the I2C to allow the clock to be fully activated
}

/** Default timeout value for read operations.
 * Set this to 0 to disable timeout detection.
 */
uint16_t I2Cdev::readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;

