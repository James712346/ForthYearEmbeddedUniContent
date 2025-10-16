#include <stdbool.h>
#include <stdint.h>
#include "i2c.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "inc/hw_memmap.h"
#include "sensor_task.h"
#include "utils/uartstdio.h"



#define VL53L0X_DEFAULT_ADDRESS (0x29)
#define I2C_BASE I2C2_BASE
#define TIMEOUT_MS (100) // Timeout for I2C operations in milliseconds

extern SemaphoreHandle_t xI2CSemaphore;


typedef enum
{
    ADDR_SIZE_8BIT,
    ADDR_SIZE_16BIT
} addr_size_t;

typedef enum
{
    REG_SIZE_8BIT,
    REG_SIZE_16BIT,
    REG_SIZE_32BIT
} reg_size_t;

bool i2c_read_register(uint8_t slave_addr, uint16_t reg_addr, addr_size_t addr_size, uint8_t *data, reg_size_t reg_size)
{
    // 1. Set slave addr for write
    I2CMasterSlaveAddrSet(I2C_BASE, slave_addr, false);

    // 2. Send the register address (8-bit or 16-bit)
    if (addr_size == ADDR_SIZE_8BIT)
    {
        I2CMasterDataPut(I2C_BASE, reg_addr & 0xFF);
        I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(TIMEOUT_MS)) != pdPASS) return false;
    }
    else if (addr_size == ADDR_SIZE_16BIT)
    {
        // Send MSB
        I2CMasterDataPut(I2C_BASE, (reg_addr >> 8) & 0xFF);
        I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(TIMEOUT_MS)) != pdPASS) return false;

        // Send LSB
        I2CMasterDataPut(I2C_BASE, reg_addr & 0xFF);
        I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(TIMEOUT_MS)) != pdPASS) return false;
    }

    // 3. Repeated start, switch to read mode
    I2CMasterSlaveAddrSet(I2C_BASE, slave_addr, true);

    int byte_count = (reg_size == REG_SIZE_8BIT) ? 1 :
                     (reg_size == REG_SIZE_16BIT) ? 2 :
                     (reg_size == REG_SIZE_32BIT) ? 4 : 0;

    if (byte_count == 0) return false;

    // 4. Receive bytes MSB → LSB
    for (int i = 0; i < byte_count; i++) {
        if (i == 0)
            I2CMasterControl(I2C_BASE, (byte_count == 1) ? I2C_MASTER_CMD_SINGLE_RECEIVE : I2C_MASTER_CMD_BURST_RECEIVE_START);
        else if (i == byte_count - 1)
            I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        else
            I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);

        if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(TIMEOUT_MS)) != pdPASS) return false;

        // MSB first
        data[byte_count - 1 - i] = I2CMasterDataGet(I2C_BASE);
    }

    return true;
}

bool i2c_read_addr8_data8(uint8_t addr, uint8_t *data)
{
    return i2c_read_register(VL53L0X_DEFAULT_ADDRESS, addr, ADDR_SIZE_8BIT, data, REG_SIZE_8BIT);
}



bool i2c_read_addr8_data16(uint8_t adr, uint16_t *data)
{
    I2CMasterSlaveAddrSet(I2C_BASE, VL53L0X_DEFAULT_ADDRESS, false);
    I2CMasterDataPut(I2C_BASE, adr);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(TIMEOUT_MS)) != pdPASS) return false;

    I2CMasterSlaveAddrSet(I2C_BASE, VL53L0X_DEFAULT_ADDRESS, true);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(TIMEOUT_MS)) != pdPASS) return false;
    uint8_t msb = I2CMasterDataGet(I2C_BASE);

    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(TIMEOUT_MS)) != pdPASS) return false;
    uint8_t lsb = I2CMasterDataGet(I2C_BASE);

    *data = ((uint16_t)msb << 8) | lsb;
    return true;
}

bool i2c_read_addr16_data8(uint16_t addr, uint8_t *data)
{
    uint8_t hi = (addr >> 8) & 0xFF;
    uint8_t lo = addr & 0xFF;

    I2CMasterSlaveAddrSet(I2C_BASE, VL53L0X_DEFAULT_ADDRESS, false);
    I2CMasterDataPut(I2C_BASE, hi);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(TIMEOUT_MS)) != pdPASS) return false;

    I2CMasterDataPut(I2C_BASE, lo);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(TIMEOUT_MS)) != pdPASS) return false;

    I2CMasterSlaveAddrSet(I2C_BASE, VL53L0X_DEFAULT_ADDRESS, true);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(TIMEOUT_MS)) != pdPASS) return false;

    *data = I2CMasterDataGet(I2C_BASE);
    return true;
}

bool i2c_read_addr16_data16(uint16_t adr, uint16_t *data)
{
    uint8_t reg_hi = adr >> 8;
    uint8_t reg_lo = adr & 0xFF;

    I2CMasterSlaveAddrSet(I2C_BASE, VL53L0X_DEFAULT_ADDRESS, false);
    I2CMasterDataPut(I2C_BASE, reg_hi);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(TIMEOUT_MS)) != pdPASS) return false;

    I2CMasterDataPut(I2C_BASE, reg_lo);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(TIMEOUT_MS)) != pdPASS) return false;

    I2CMasterSlaveAddrSet(I2C_BASE, VL53L0X_DEFAULT_ADDRESS, true);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(TIMEOUT_MS)) != pdPASS) return false;
    uint8_t msb = I2CMasterDataGet(I2C_BASE);

    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(TIMEOUT_MS)) != pdPASS) return false;
    uint8_t lsb = I2CMasterDataGet(I2C_BASE);

    *data = ((uint16_t)msb << 8) | lsb;
    return true;
}

bool i2c_read_addr8_data32(uint16_t addr, uint32_t *data)
{
    uint8_t bytes[4];
    if (!i2c_read_addr8_bytes((uint8_t)addr, bytes, 4)) return false;
    *data = ((uint32_t)bytes[0] << 24) | ((uint32_t)bytes[1] << 16) | ((uint32_t)bytes[2] << 8) | bytes[3];
    return true;
}

bool i2c_read_addr16_data32(uint16_t addr, uint32_t *data)
{
    uint8_t addr_hi = addr >> 8;
    uint8_t addr_lo = addr & 0xFF;
    uint8_t bytes[4];

    I2CMasterSlaveAddrSet(I2C_BASE, VL53L0X_DEFAULT_ADDRESS, false);
    I2CMasterDataPut(I2C_BASE, addr_hi);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(TIMEOUT_MS)) != pdPASS) return false;

    I2CMasterDataPut(I2C_BASE, addr_lo);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(TIMEOUT_MS)) != pdPASS) return false;

    return i2c_read_addr8_bytes(0x00, bytes, 4) &&
           ((*data = ((uint32_t)bytes[0] << 24 | (uint32_t)bytes[1] << 16 |
                      (uint32_t)bytes[2] << 8 | bytes[3])), true);
}

bool i2c_read_addr8_bytes(uint8_t start_addr, uint8_t *bytes, uint16_t byte_count)
{
    I2CMasterSlaveAddrSet(I2C_BASE, VL53L0X_DEFAULT_ADDRESS, false);
    I2CMasterDataPut(I2C_BASE, start_addr);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(TIMEOUT_MS)) != pdPASS) return false;

    I2CMasterSlaveAddrSet(I2C_BASE, VL53L0X_DEFAULT_ADDRESS, true);

    for (uint16_t i = 0; i < byte_count; i++)
    {
        if (i == 0)
            I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
        else if (i == byte_count - 1)
            I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        else
            I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);

        if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(TIMEOUT_MS)) != pdPASS) return false;
        bytes[i] = I2CMasterDataGet(I2C_BASE);
    }

    return true;
}


bool i2c_write_addr8_data8(uint8_t addr, uint8_t value)
{
    I2CMasterSlaveAddrSet(I2C_BASE, VL53L0X_DEFAULT_ADDRESS, false); /* Set slave address */
    I2CMasterDataPut(I2C_BASE, addr); /* Set register address */
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START); /* Send start condition */
    if(xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(100)) != pdPASS)
    {
        // UARTprintf("Timeout waiting for I2C write\n");
        return false;
    }
    if(I2CMasterErr(I2C_BASE) != I2C_MASTER_ERR_NONE)return false; /* Check for errors */

    I2CMasterDataPut(I2C_BASE, value); /* Set data to be written */
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH); /* Send stop condition */
    if(xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(100)) != pdPASS)
    {
        // UARTprintf("Timeout waiting for I2C write\n");
        return false;
    }
    return I2CMasterErr(I2C_BASE) == I2C_MASTER_ERR_NONE; /* Check for errors */
}

bool i2c_write_addr8_data16(uint8_t addr, uint16_t value)
{
    uint8_t bytes[] = { (value >> 8) & 0xFF, value & 0xFF };
    return i2c_write_addr8_bytes(addr, bytes, 2);
}

bool i2c_write_addr16_data8(uint16_t addr, uint8_t value)
{
    uint8_t bytes[] = { (addr >> 8) & 0xFF, addr & 0xFF, value };
    return i2c_write_addr8_bytes(0x00, bytes, 3);
}

bool i2c_write_addr16_data16(uint16_t addr, uint16_t value)
{
    uint8_t bytes[] = { (addr >> 8) & 0xFF, addr & 0xFF, (value >> 8) & 0xFF, value & 0xFF };
    return i2c_write_addr8_bytes(0x00, bytes, 4);
}

bool i2c_write_addr8_bytes(uint8_t start_addr, uint8_t *bytes, uint16_t byte_count)
{
    I2CMasterSlaveAddrSet(I2C_BASE, VL53L0X_DEFAULT_ADDRESS, false);
    I2CMasterDataPut(I2C_BASE, start_addr);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(TIMEOUT_MS)) != pdPASS) return false;

    for (uint16_t i = 0; i < byte_count; i++)
    {
        I2CMasterDataPut(I2C_BASE, bytes[i]);
        I2CMasterControl(I2C_BASE,
            i == byte_count - 1 ? I2C_MASTER_CMD_BURST_SEND_FINISH : I2C_MASTER_CMD_BURST_SEND_CONT);
        if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(TIMEOUT_MS)) != pdPASS) return false;
    }

    return true;
}

void i2c_set_slave_address(uint8_t addr)
{
    I2CMasterSlaveAddrSet(I2C2_BASE, addr, false); /* Set slave address */
}

// void i2c_init()
// {
//     /* Pinmux P1.6 (SCL) and P1.7 (SDA) to I2C peripheral  */
//     P1SEL |= BIT6 + BIT7;
//     P1SEL2 |= BIT6 + BIT7;

//     UCB0CTL1 |= UCSWRST; /* Enable SW reset */
//     UCB0CTL0 = UCMST + UCSYNC + UCMODE_3; /* Single master, synchronous mode, I2C mode */
//     UCB0CTL1 |= UCSSEL_2; /* SMCLK */
//     UCB0BR0 = 10; /* SMCLK / 10 = ~100kHz */
//     UCB0BR1 = 0;
//     UCB0CTL1 &= ~UCSWRST; /* Clear SW */
//     i2c_set_slave_address(DEFAULT_SLAVE_ADDRESS);
// }
