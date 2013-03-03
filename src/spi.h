#ifndef SPI_H
#define SPI_H

#include "stm32f30x.h"
#include "stm32f30x_bitmask.h"
#include "gpio.h"

inline uint16_t spi_rw16_single(SPI_TypeDef *spi, uint16_t data, GPIOPin pin)
{
    GPIO::clear(pin);
    spi->DR = data;
    while((SPI1->SR & (SPI_SR_RXNE_gm | SPI_SR_TXE_gm)) != (SPI_SR_RXNE_gm | SPI_SR_TXE_gm)) {}
    GPIO::set(pin);

    return SPI1->DR;
}

inline uint16_t spi_rw16_mul_begin(SPI_TypeDef *spi, uint16_t data, GPIOPin pin)
{
    GPIO::clear(pin);
    spi->DR = data;
    while((SPI1->SR & (SPI_SR_RXNE_gm | SPI_SR_TXE_gm)) != (SPI_SR_RXNE_gm | SPI_SR_TXE_gm)) {}

    return SPI1->DR;
}

inline uint16_t spi_rw16_mul_next(SPI_TypeDef *spi, uint16_t data)
{
    spi->DR = data;
    while((SPI1->SR & (SPI_SR_RXNE_gm | SPI_SR_TXE_gm)) != (SPI_SR_RXNE_gm | SPI_SR_TXE_gm)) {}

    return SPI1->DR;
}

inline uint16_t spi_rw16_mul_end(SPI_TypeDef *spi, uint16_t data, GPIOPin pin)
{
    spi->DR = data;
    while((SPI1->SR & (SPI_SR_RXNE_gm | SPI_SR_TXE_gm)) != (SPI_SR_RXNE_gm | SPI_SR_TXE_gm)) {}
    GPIO::set(pin);

    return SPI1->DR;
}

#endif