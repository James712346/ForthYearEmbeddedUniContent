/**************************************************************************************************
*  Filename:       i2cOptDriver.c
*  By:             Jesse Haviland
*  Created:        1 February 2019
*  Revised:        23 March 2019
*  Revision:       2.0
*
*  Description:    i2c Driver for use with opt3001.c and the TI OP3001 Optical Sensor
*************************************************************************************************/

// ----------------------- Includes -----------------------
#include "i2cOptDriver.h"
#include "inc/hw_memmap.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "driverlib/sysctl.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern SemaphoreHandle_t xI2CSemaphore;
extern SemaphoreHandle_t xI2CBusSemaphore;

// Interrupt for I2C 2
void I2C0IntHandler(void) {
    BaseType_t woken = pdFALSE;
    uint32_t status = I2CMasterIntStatusEx(I2C2_BASE, true);

    // signal on DATA or STOP only
    if (status & I2C_MASTER_INT_DATA) {
        xSemaphoreGiveFromISR(xI2CSemaphore, &woken);
    }
    if (status & I2C_MASTER_INT_STOP) {
        xSemaphoreGiveFromISR(xI2CSemaphore, &woken);
    }

    // clear whatever caused the interrupt
    I2CMasterIntClearEx(I2C2_BASE, status);
    portYIELD_FROM_ISR(woken);
}




/*
 * Sets slave address to ui8Addr
 * Writes ui8Reg over I2C to specify register being read from
 * Reads two bytes from I2C slave, using three interrupts to match
 * semaphore takes: START, CONTINUE, FINISH.
 * Stores the two received bytes into data[0] and data[1].
 */


bool writeI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{   
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100);
    bool success = true;
    
    if (xSemaphoreTake(xI2CSemaphore, xMaxBlockTime) != pdTRUE) {
        return false;
    }
    
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false);

    I2CMasterDataPut(I2C2_BASE, ui8Reg);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    
    if (xSemaphoreTake(xI2CSemaphore, xMaxBlockTime) != pdTRUE) {
        xSemaphoreGive(xI2CSemaphore);
        return false;
    }
    
    I2CMasterDataPut(I2C2_BASE, data[0]);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

    if (xSemaphoreTake(xI2CSemaphore, xMaxBlockTime) != pdTRUE) {
        xSemaphoreGive(xI2CSemaphore); 
        return false;
    }

    I2CMasterDataPut(I2C2_BASE, data[1]);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    
    
    return success;
}

bool readI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *data)
{
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100);
    bool success = true;
    
    if (xSemaphoreTake(xI2CSemaphore, xMaxBlockTime) != pdTRUE) {
        return false; 
    }
    
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, false);
    I2CMasterDataPut(I2C2_BASE, ui8Reg);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    
    if (xSemaphoreTake(xI2CSemaphore, xMaxBlockTime) != pdTRUE) {
        xSemaphoreGive(xI2CSemaphore); 
        return false;
    }
    
    I2CMasterSlaveAddrSet(I2C2_BASE, ui8Addr, true);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    
    if (xSemaphoreTake(xI2CSemaphore, xMaxBlockTime) != pdTRUE) {
        xSemaphoreGive(xI2CSemaphore); 
        return false;
    }
    
    data[0] = I2CMasterDataGet(I2C2_BASE);
    
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    
    if (xSemaphoreTake(xI2CSemaphore, xMaxBlockTime) != pdTRUE) {
        xSemaphoreGive(xI2CSemaphore); 
        return false;
    }
    
    data[1] = I2CMasterDataGet(I2C2_BASE);
    
    xSemaphoreGive(xI2CSemaphore);
    
    return success;
}



