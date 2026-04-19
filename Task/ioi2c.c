/*
 * ioi2c.c - HAL I2C adapter for MPU6050/DMP library
 *
 * This file replaces the original software I2C (IOI2C.c) with
 * STM32 HAL hardware I2C2 calls.
 * Original used PB14(SCL)/PB15(SDA) software I2C.
 * Target uses PB10(SCL)/PB11(SDA) hardware I2C2.
 */

#include "ioi2c.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c2;

#define I2C_TIMEOUT 100

static uint32_t i2c_err_cnt = 0;

/**************************************************************************
Function: IIC pin initialization (HAL version - already done by MX_I2C2_Init)
**************************************************************************/
void IIC_Init(void)
{
    // Hardware I2C2 is initialized by CubeMX generated MX_I2C2_Init()
    // Nothing to do here
}

/**************************************************************************
Function: i2cWrite - DMP library I2C write interface
Input   : addr: 7-bit device address, reg: register, len: length, data: buffer
Output  : 0: success, 1: fail
**************************************************************************/
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    HAL_StatusTypeDef st;
    st = HAL_I2C_Mem_Write(&hi2c2, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
    if (st == HAL_OK)
        return 0;
    i2c_err_cnt++;
    if (i2c_err_cnt <= 10)
        printf("[I2C] W FAIL #%lu: addr=0x%02X reg=0x%02X len=%d st=%d i2cErr=0x%lX\r\n",
               (unsigned long)i2c_err_cnt, addr, reg, len, (int)st,
               (unsigned long)hi2c2.ErrorCode);
    return 1;
}

/**************************************************************************
Function: i2cRead - DMP library I2C read interface
Input   : addr: 7-bit device address, reg: register, len: length, buf: buffer
Output  : 0: success, 1: fail
**************************************************************************/
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    HAL_StatusTypeDef st;
    st = HAL_I2C_Mem_Read(&hi2c2, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, I2C_TIMEOUT);
    if (st == HAL_OK)
        return 0;
    i2c_err_cnt++;
    if (i2c_err_cnt <= 10)
        printf("[I2C] R FAIL #%lu: addr=0x%02X reg=0x%02X len=%d st=%d i2cErr=0x%lX\r\n",
               (unsigned long)i2c_err_cnt, addr, reg, len, (int)st,
               (unsigned long)hi2c2.ErrorCode);
    return 1;
}

/**************************************************************************
Function: Read one byte from device register
Input   : I2C_Addr: 8-bit device address (already shifted), addr: register
Output  : data read
**************************************************************************/
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr, unsigned char addr)
{
    unsigned char res = 0;
    HAL_I2C_Mem_Read(&hi2c2, I2C_Addr, addr, I2C_MEMADD_SIZE_8BIT, &res, 1, I2C_TIMEOUT);
    return res;
}

/**************************************************************************
Function: IIC read multiple bytes
Input   : dev: 8-bit device address, reg: register, length: count, data: buffer
Output  : count of bytes read
**************************************************************************/
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
    if (HAL_I2C_Mem_Read(&hi2c2, dev, reg, I2C_MEMADD_SIZE_8BIT, data, length, I2C_TIMEOUT) == HAL_OK)
        return length;
    return 0;
}

/**************************************************************************
Function: IIC write multiple bytes
Input   : dev: 8-bit device address, reg: register, length: count, data: buffer
Output  : 1
**************************************************************************/
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{
    HAL_I2C_Mem_Write(&hi2c2, dev, reg, I2C_MEMADD_SIZE_8BIT, data, length, I2C_TIMEOUT);
    return 1;
}

/**************************************************************************
Function: IIC read one byte
Input   : dev: 8-bit device address, reg: register, data: pointer to store
Output  : 1
**************************************************************************/
uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data)
{
    *data = I2C_ReadOneByte(dev, reg);
    return 1;
}

/**************************************************************************
Function: IIC write one byte
Input   : dev: 8-bit device address, reg: register, data: value
Output  : 1
**************************************************************************/
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
{
    return IICwriteBytes(dev, reg, 1, &data);
}

/**************************************************************************
Function: Read-modify-write multiple bits in a register byte
Input   : dev: 8-bit address, reg: register, bitStart: MSB of field,
          length: field width, data: value to write into field
Output  : 1: success, 0: fail
**************************************************************************/
uint8_t IICwriteBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data)
{
    uint8_t b;
    if (IICreadByte(dev, reg, &b) != 0) {
        uint8_t mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}

/**************************************************************************
Function: Read-modify-write one bit in a register byte
Input   : dev: 8-bit address, reg: register, bitNum: bit position,
          data: 0 clears, non-zero sets
Output  : 1
**************************************************************************/
uint8_t IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data)
{
    uint8_t b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}
