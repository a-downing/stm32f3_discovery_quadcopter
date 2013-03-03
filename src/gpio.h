#ifndef GPIO_H
#define GPIO_H

#include "stm32f30x.h"
#include "stm32f30x_bitmask.h"

constexpr uint16_t _pindef(uint8_t port, uint8_t pin)
{
    return port << 8 | pin;
}

enum GPIOPort
{
    PA = 0,
    PB,
    PC,
    PD,
    PE,
    PF
};

enum GPIOPin : uint16_t
{
    
PA0=0x0000, PA1=0x0001, PA2=0x0002, PA3=0x0003, PA4=0x0004, PA5=0x0005, PA6=0x0006, PA7=0x0007, PA8=0x0008, PA9=0x0009, PA10=0x000A, PA11=0x000B, PA12=0x000C, PA13=0x000D, PA14=0x000E, PA15=0x000F, 
PB0=0x0100, PB1=0x0101, PB2=0x0102, PB3=0x0103, PB4=0x0104, PB5=0x0105, PB6=0x0106, PB7=0x0107, PB8=0x0108, PB9=0x0109, PB10=0x010A, PB11=0x010B, PB12=0x010C, PB13=0x010D, PB14=0x010E, PB15=0x010F, 
PC0=0x0200, PC1=0x0201, PC2=0x0202, PC3=0x0203, PC4=0x0204, PC5=0x0205, PC6=0x0206, PC7=0x0207, PC8=0x0208, PC9=0x0209, PC10=0x020A, PC11=0x020B, PC12=0x020C, PC13=0x020D, PC14=0x020E, PC15=0x020F, 
PD0=0x0300, PD1=0x0301, PD2=0x0302, PD3=0x0303, PD4=0x0304, PD5=0x0305, PD6=0x0306, PD7=0x0307, PD8=0x0308, PD9=0x0309, PD10=0x030A, PD11=0x030B, PD12=0x030C, PD13=0x030D, PD14=0x030E, PD15=0x030F, 
PE0=0x0400, PE1=0x0401, PE2=0x0402, PE3=0x0403, PE4=0x0404, PE5=0x0405, PE6=0x0406, PE7=0x0407, PE8=0x0408, PE9=0x0409, PE10=0x040A, PE11=0x040B, PE12=0x040C, PE13=0x040D, PE14=0x040E, PE15=0x040F, 
PF0=0x0500, PF1=0x0501, PF2=0x0502, PF3=0x0503, PF4=0x0504, PF5=0x0505, PF6=0x0506, PF7=0x0507, PF8=0x0508, PF9=0x0509, PF10=0x050A, PF11=0x050B, PF12=0x050C, PF13=0x050D, PF14=0x050E, PF15=0x050F, 
};

constexpr uint8_t _pin(GPIOPin pin)
{
    return uint16_t(pin) & 0xFF;
}

constexpr uint8_t _port(GPIOPin pin)
{
    return uint16_t(pin) >> 8;
}

enum GPIOMode
{
    IN = 0x0,
    OUT = 0x1,
    AF = 0x2,
    ANALOG = 0x3
};

enum GPIOSpeed
{
    _2MHz = 0x0,
    _10MHz = 0x1,
    _50MHz = 0x3
};

enum GPIOOutputType
{
    PUSH_PULL = 0x0,
    OPEN_DRAIN = 0x1
};

enum GPIOPullUpDown
{
    NONE = 0x0,
    PULL_UP = 0x1,
    PULL_DOWN = 0x2
};

enum AFNumber
{
    AF0=0, AF1, AF2, AF3,
    AF4, AF5, AF6, AF7,
    AF8, AF9, AF10, AF11,
    AF12, AF13, AF14, AF15
};

class GPIO
{
    static constexpr GPIO_TypeDef * const gpio[6] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF};

public:
    static void enable(GPIOPort port)
    {
        RCC->AHBENR |= 0x00020000 << int(port);
    }

    static void setSpeed(GPIOPin pin, GPIOSpeed speed)
    {
        gpio[_port(pin)]->OSPEEDR = (gpio[_port(pin)]->OSPEEDR & ~(0x3 << (_pin(pin) * 2))) | (uint32_t(speed) << (_pin(pin) * 2));
    }

    static void setMode(GPIOPin pin, GPIOMode mode)
    {
        gpio[_port(pin)]->MODER = (gpio[_port(pin)]->MODER & ~(0x3 << (_pin(pin) * 2))) | (uint32_t(mode) << (_pin(pin) * 2));
    }

    static void setOutputType(GPIOPin pin, GPIOOutputType otype)
    {
        gpio[_port(pin)]->OTYPER = (gpio[_port(pin)]->OTYPER & ~(0x3 << (_pin(pin) * 2))) | (uint32_t(otype) << (_pin(pin) * 2));
    }

    static void setPullUpDown(GPIOPin pin, GPIOPullUpDown pupd)
    {
        gpio[_port(pin)]->PUPDR = (gpio[_port(pin)]->PUPDR & ~(0x3 << (_pin(pin) * 2))) | (uint32_t(pupd) << (_pin(pin) * 2));
    }

    static void setAF(GPIOPin pin, AFNumber af)
    {
        gpio[_port(pin)]->AFR[(_pin(pin) < 8) ? 0 : 1] =
            (gpio[_port(pin)]->AFR[(_pin(pin) < 8) ? 0 : 1] & ~(0xF << (_pin(pin) * 4)))
            | (int(af) << (((_pin(pin) < 8) ? _pin(pin) : _pin(pin) - 8) * 4))
        ;
    }

    static uint16_t read(GPIOPort port)
    {
        return gpio[int(port)]->IDR;
    }

    static uint8_t read(GPIOPin pin)
    {
        return (gpio[_port(pin)]->IDR >> _pin(pin)) & 0x1;
    }

    static void write(GPIOPort port, uint16_t val)
    {
        gpio[int(port)]->IDR = val;
    }

    static void write(GPIOPin pin, uint8_t val)
    {
        gpio[_port(pin)]->IDR = (gpio[_port(pin)]->IDR & ~(0x1 << _pin(pin))) | (val & 0x1);
    }

    static void set(GPIOPort port, uint16_t bits)
    {
        gpio[int(port)]->BSRR = bits;
    }

    static void clear(GPIOPort port, uint16_t bits)
    {
        gpio[int(port)]->BSRR = bits << 16;
    }

    static void set(GPIOPin pin)
    {
        gpio[_port(pin)]->BSRR = 1 << _pin(pin);
    }

    static void clear(GPIOPin pin)
    {
        gpio[_port(pin)]->BSRR = (1 << _pin(pin)) << 16;
    }
};

#endif