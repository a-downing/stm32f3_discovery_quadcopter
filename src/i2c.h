#ifndef I2C_H
#define I2C_H

#include "stm32f30x.h"
#include "stm32f30x_bitmask.h"

inline void i2c_init(I2C_TypeDef *i2c)
{
    // set I2C1 clock to PCLCK (72MHz)
    RCC->CFGR3 |= RCC_CFGR3_I2C1SW;

    // enable I2C1 clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    asm volatile("dmb");

    // disable analog filter
    i2c->CR1 |= I2C_CR1_ANFOFF;

    // from stm32f3_i2c_calc.py (400KHz, 125ns rise/fall time, no AF/DFN)
    uint8_t sdadel = 7;
    uint8_t scldel = 5;
    i2c->TIMINGR = 0x30000C19 | ((scldel & 0x0F) << 20) | ((sdadel & 0x0F) << 16);

    // enable I2C1
    i2c->CR1 |= I2C_CR1_PE;
}

inline void i2c_set_addr(I2C_TypeDef *i2c, uint8_t addr)
{
    i2c->CR2 = (i2c->CR2 & ~I2C_CR2_SADD_gm) | addr;
}

inline void i2c_start_write(I2C_TypeDef *i2c, int nbytes)
{
    i2c->CR2 =
        (i2c->CR2 & ~(I2C_CR2_NBYTES_gm | I2C_CR2_RD_WRN_gm))
        | (nbytes << I2C_CR2_NBYTES_gp)
        | I2C_CR2_START_gm
    ;
    while(i2c->CR2 & I2C_CR2_START) {}
}

inline void i2c_start_read(I2C_TypeDef *i2c, int nbytes)
{
    i2c->CR2 =
        (i2c->CR2 & ~(I2C_CR2_NBYTES_gm))
        | (nbytes << I2C_CR2_NBYTES_gp)
        | I2C_CR2_RD_WRN_gm
        | I2C_CR2_START_gm
    ;
    while(i2c->CR2 & I2C_CR2_START_gm) {}
}

inline void i2c_stop(I2C_TypeDef *i2c)
{
    i2c->CR2 |= I2C_CR2_STOP_gm;
    while(i2c->CR2 & I2C_CR2_STOP_gm) {}
}

inline void i2c_write(I2C_TypeDef *i2c, uint8_t data)
{
    i2c->TXDR = data;
    while(!(i2c->ISR & I2C_ISR_TXE)) {}
}

inline uint8_t i2c_read(I2C_TypeDef *i2c)
{
    while(!(i2c->ISR & I2C_ISR_RXNE_gm)) {}
    return i2c->RXDR & 0xFF;
}

#endif