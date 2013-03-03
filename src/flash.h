#ifndef FLASH_H
#define FLASH_H

#include "stm32f30x.h"
#include "stm32f30x_bitmask.h"

#define FLASH_DATA_BASE ((uint32_t)0x0803F800)
#define FLASH_DATA_SIZE ((uint32_t)2048)

class Flash
{
public:
    static void *page_to_addr(uint8_t page)
    {
        return (void *)(FLASH_BASE + page * 2048);
    }

    static bool unlock()
    {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;

        return !(FLASH->CR & FLASH_CR_LOCK);
    }

    static void lock()
    {
        FLASH->CR |= FLASH_CR_LOCK;
    }

    static bool erasePage(uint32_t *addr)
    {
        while(FLASH->SR & FLASH_SR_BSY) {}

        FLASH->CR |= FLASH_CR_PER;
        FLASH->AR = (uint32_t)addr;
        FLASH->CR |= FLASH_CR_STRT;

        asm volatile("nop");
        while(FLASH->SR & FLASH_SR_BSY) {}

        FLASH->CR &= ~FLASH_CR_PER;

        bool ret = FLASH->SR == FLASH_SR_EOP;

        // clear bits
        FLASH->SR |= FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPERR;

        return ret;
    }

    static bool erasePage(uint8_t page)
    {
        return erasePage((uint32_t *)(FLASH_BASE + page * 2048));
    }

    static bool program(volatile uint16_t *addr, uint16_t val)
    {
        while(FLASH->SR & FLASH_SR_BSY) {}

        FLASH->CR |= FLASH_CR_PG;
        
        *addr = val;

        asm volatile("nop");
        while(FLASH->SR & FLASH_SR_BSY) {}

        FLASH->CR &= ~FLASH_CR_PG;

        bool ret = FLASH->SR == FLASH_SR_EOP;

        // clear bits
        FLASH->SR |= FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPERR;

        return ret;
    }
};

#endif