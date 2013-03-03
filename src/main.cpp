#include <algorithm>

#include "stm32f30x.h"
#include "stm32f30x_bitmask.h"

#include "lsm303dlhc.h"
#include "l3gd20.h"
#include "gpio.h"
#include "spi.h"
#include "i2c.h"
#include "flash.h"

#include "f3usb.h"
#include "usbdevice.h"
#include "descriptor.h"

#include "quadcopter_math.h"
#include "pid.h"

// IN/OUT is always relative to host
auto dev_desc = device_desc(0x200, 0, 0, 0, 64, 0xDEAD, 0xBEEF, 0, 0, 0, 0, 1);
auto config_desc = configuration_desc(1, 1, 0, 0xc0, 0,
    interface_desc(0, 0, 3, 0, 0x00, 0x00, 0,
        endpoint_desc(0x01, 0x02, 64, 0),  // settings OUT
        endpoint_desc(0x81, 0x02, 64, 0),  // settings IN
        endpoint_desc(0x82, 0x02, 64, 0)  // sensor data IN
    )
);

const float dt = 1.0f / 380.0f; // change this to use a timer or something to get actual dt
const Vec3f UP_AXIS(0.0f, 0.0f, 1.0f);
const Vec3f RIGHT_AXIS(0.0f, 1.0f, 0.0f);
const Vec3f FWD_AXIS(1.0f, 0.0f, 0.0f);

using namespace USB;

volatile uint32_t g_counter = 0;

struct FlightSettings
{
    enum Operation
    {
        WRITE_FLASH = (1 << 0),
        ZERO_ACCEL = (1 << 1),
        ZERO_GYRO = (1 << 2)
    };

    uint32_t op;

    float filt_c = 0.99f;

    float servo_pulse_min_in = 0.001f;
    float servo_pulse_max_in = 0.002f;

    float servo_pulse_min_out = 0.001f;
    float servo_pulse_max_out = 0.002f;

    float pitch_range = pi / 4.0f; // +/-45 deg
    float roll_range = pi / 4.0f; // +/-45 deg
    float yaw_range = pi * 2.0f; // +/- 360deg/s

    float throttle_max = 0.9f;
    float throttle_min_ctrl = 0.1; // won't PID control below this

    float esc_trim[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    PIDProperties pitch_prop = {0.125f, 0.0f, 0.0f, 0.0f};
    PIDProperties roll_prop = {0.125f, 0.0f, 0.0f, 0.0f};
    PIDProperties yaw_prop = {0.0125f, 0.0f, 0.0f, 0.0f};

    Vec3f gyro_zero;
    Quatf accel_zero;

    FlightSettings()
    {
        // FLASH_DATA_BASE is reserved in linker script (last 2K page of flash)
        volatile uint32_t *ptr = (uint32_t *)FLASH_DATA_BASE;

        // the flash has been programmed, use the data there instead of defaults
        if(*ptr != 0xFFFFFFFF)
        {
            *this = *((FlightSettings *)ptr);
        }
    }

    void flash()
    {
        // flash needs to be written 16 bits at a time

        const int num_writes = sizeof(*this) / sizeof(uint16_t) + sizeof(*this) % sizeof(uint16_t);

        Flash::unlock();

        Flash::erasePage((uint32_t *)FLASH_DATA_BASE);

        for(int i = 0; i < num_writes; ++i)
        {
            uint16_t *flash = (uint16_t *)FLASH_DATA_BASE;
            uint8_t *settings8 = (uint8_t *)this;
            uint16_t *settings16 = (uint16_t *)this;
            uint16_t data = (i * sizeof(uint16_t) < sizeof(*this) - 1) ? settings16[i] : settings8[i * 2];

            Flash::program(flash + i, data);
        }

        Flash::lock();
    }

    void loadFromFlash()
    {

        // FLASH_DATA_BASE is reserved in linker script (last 2K page of flash)
        volatile uint32_t *ptr = (uint32_t *)FLASH_DATA_BASE;

        // just do it
        *this = *((FlightSettings *)ptr);
    }
};

//static_assert(sizeof(FlightSettings) == 136, "FlightSettings struct wrong size");
static_assert(sizeof(FlightSettings) <= FLASH_DATA_SIZE, "FlightSettings struct is too large to fit in FLASH_DATA section.");
static_assert(alignof(FlightSettings) >= sizeof(uint16_t), "Aligntment of FlightSettings struct should be at least sizeof(uint16_t) in order to correctly program to flash");

FlightSettings g_settings;

struct FlightControl
{
    Vec3f up = Vec3f(0.0f, 0.0f, 1.0f);
    Vec3f gyro_filtered = Vec3f(0.0f, 0.0f, 0.0f);
    Vec3f accel_filtered = Vec3f(0.0f, 0.0f, 1.0f);

    PIDController pitch_ctrl;
    PIDController roll_ctrl;
    PIDController yaw_ctrl;

    FlightControl() = delete;
    FlightControl(const FlightSettings& settings) : pitch_ctrl(settings.pitch_prop), roll_ctrl(settings.roll_prop), yaw_ctrl(settings.yaw_prop) {}
};

FlightControl g_flightctrl(g_settings);

struct Telemetry
{
    Vec3f gyro;
    Vec3f accel;
    
    float pitch;
    float roll;
    float yaw;

    float cmd_throttle;
    float cmd_pitch;
    float cmd_roll;
    float cmd_yaw;

    float pitch_correct;
    float roll_correct;
    float yaw_correct;

    struct ReceiverChannel
    {
        float period;
        float pulse_width;
    };

    ReceiverChannel rx_chan[4];
};

Telemetry g_telemetry;

class MyUSBDeviceDriver : public USBDeviceDriver
{
    enum
    {
        SETTINGS_OUT_EP = 0x01,
        SETTINGS_IN_EP = 0x81,
        SENSORS_IN_EP = 0x82,
        INTERFACE_NUM = 0,
        NUM_ENDPOINTS = 3
    };

    class DeviceState
    {
    public:
        struct EndpointState
        {
            uint32_t bytes_rx;
            uint32_t bytes_tx;
        };

        void reset()
        {
            for(size_t i = 0; i < sizeof(ep_state) / sizeof(EndpointState); ++i)
            {
                ep_state[i] = {0, 0};
            }
        }

        EndpointState& getEPState(uint8_t ep)
        {
            return ep_state[ep & 0x0F];
        }

    private:
        EndpointState ep_state[NUM_ENDPOINTS] = {{0, 0}, {0, 0}, {0, 0}};
    };

    DeviceState dev_state;
    uint32_t busy_eps = 0;
    uint8_t recv_buf[sizeof(FlightSettings)];

    void ep_busy(uint8_t ep)
    {
        busy_eps |= ((0x1 << ((ep & 0x0F) * 2)) << ((ep & 0x80) >> 7));
    }

    void ep_idle(uint8_t ep)
    {
        busy_eps &= ~((0x1 << ((ep & 0x0F) * 2)) << ((ep & 0x80) >> 7));
    }

public:
    MyUSBDeviceDriver(const uint8_t *d_desc, const uint8_t *c_desc, USBHardwareDriver *drv) : USBDeviceDriver(d_desc, c_desc, drv) {}  

    virtual void init() override
    {
        USBDeviceDriver::init();
    }

    virtual void process() override
    {
        USBDeviceDriver::process();
    }

    bool busy()
    {
        return busy_eps != 0;
    }

    virtual void onWriteComplete(uint8_t ep) override
    {
        if(dev_state.getEPState(ep).bytes_tx > 0)
        {
            uint32_t max_packet_size =
                getConfigurationDescriptor()
                .getInterfaceByNum(INTERFACE_NUM)
                .getEndpointByAddress(ep)
                .getMaxPacketSize()
            ;

            if(ep == SENSORS_IN_EP)
            {
                uint32_t write_size = std::min((uint32_t)sizeof(g_telemetry) - dev_state.getEPState(ep).bytes_tx, max_packet_size);
                write(ep, (uint8_t *)&g_telemetry + dev_state.getEPState(ep).bytes_tx, write_size);
                dev_state.getEPState(ep).bytes_tx += write_size;

                if(dev_state.getEPState(ep).bytes_tx == sizeof(g_telemetry))
                {
                    dev_state.getEPState(ep).bytes_tx = 0;
                    ep_idle(ep);
                }
            }
            else if(ep == SETTINGS_IN_EP)
            {
                uint32_t write_size = std::min((uint32_t)sizeof(g_settings) - dev_state.getEPState(ep).bytes_tx, max_packet_size);
                write(ep, (uint8_t *)&g_settings + dev_state.getEPState(ep).bytes_tx, write_size);
                dev_state.getEPState(ep).bytes_tx += write_size;

                if(dev_state.getEPState(ep).bytes_tx == sizeof(g_settings))
                {
                    dev_state.getEPState(ep).bytes_tx = 0;
                }
            }
        }
        else // if(dev_state.getEPState(ep).bytes_tx == 0)
        {
            if(ep == SETTINGS_IN_EP)
            {
                ep_idle(ep);
                writeFlightSettings();
            }
        }
    }

    virtual void onReadReady(uint8_t ep, uint32_t len) override
    {
        if(ep == SETTINGS_OUT_EP)
        {
            uint32_t max_packet_size =
                getConfigurationDescriptor()
                .getInterfaceByNum(INTERFACE_NUM)
                .getEndpointByAddress(ep)
                .getMaxPacketSize()
            ;

            uint32_t read_size = std::min((uint32_t)sizeof(g_settings) - dev_state.getEPState(ep).bytes_rx, max_packet_size);

            read(ep, recv_buf + dev_state.getEPState(ep).bytes_rx, read_size);
            dev_state.getEPState(ep).bytes_rx += read_size;
            ep_busy(ep);

            if(dev_state.getEPState(ep).bytes_rx == sizeof(g_settings))
            {
                g_settings = *((FlightSettings *)recv_buf);

                if(g_settings.op & g_settings.ZERO_GYRO)
                {
                    g_settings.gyro_zero = g_flightctrl.gyro_filtered;
                }

                if(g_settings.op & g_settings.ZERO_ACCEL)
                {
                    Vec3f rot_axis = g_flightctrl.accel_filtered.cross(UP_AXIS);
                    rot_axis.normalize();
                    float rot_dot = g_flightctrl.accel_filtered.dot(UP_AXIS);
                    g_settings.accel_zero = Quatf(rot_axis, acosf(rot_dot));
                }

                if(g_settings.op & g_settings.WRITE_FLASH)
                {
                    g_settings.flash();

                    // just so host can read this back to make sure nothing was corrupted.
                    g_settings.loadFromFlash();
                }

                g_flightctrl = FlightControl(g_settings);

                dev_state.getEPState(ep).bytes_rx = 0;
                ep_idle(ep);

                writeFlightSettings();
            }

            // ready for more data
            setOutTokenHandshake(ep, ACK);
        }
    }

    virtual void onSetupRequest(uint8_t ep, const SetupPacket &setup) override
    {
        write(ep, nullptr, 0);
    }

    virtual void onReset() override
    {
        USBDeviceDriver::onReset();
        dev_state.reset();

        writeFlightSettings();
    }

    void writeTelemetry()
    {
        if(dev_state.getEPState(SENSORS_IN_EP).bytes_tx == 0)
        {
            uint32_t max_packet_size =
                getConfigurationDescriptor()
                .getInterfaceByNum(INTERFACE_NUM)
                .getEndpointByAddress(SENSORS_IN_EP)
                .getMaxPacketSize()
            ;

            uint32_t write_size = std::min((uint32_t)sizeof(g_telemetry), (uint32_t)max_packet_size);
            write(SENSORS_IN_EP, (uint8_t *)&g_telemetry, write_size);
            dev_state.getEPState(SENSORS_IN_EP).bytes_tx = write_size;
            ep_busy(SENSORS_IN_EP);
        }
    }

    void writeFlightSettings()
    {
        if(dev_state.getEPState(SETTINGS_IN_EP).bytes_tx == 0)
        {
            uint32_t max_packet_size =
                getConfigurationDescriptor()
                .getInterfaceByNum(INTERFACE_NUM)
                .getEndpointByAddress(SETTINGS_IN_EP)
                .getMaxPacketSize()
            ;

            uint32_t write_size = std::min((uint32_t)sizeof(g_settings), (uint32_t)max_packet_size);
            write(SETTINGS_IN_EP, (uint8_t *)&g_settings, write_size);
            dev_state.getEPState(SETTINGS_IN_EP).bytes_tx = write_size;
            ep_busy(SETTINGS_IN_EP);
        }
    }
};

void reciever_input_init();

int main()
{
    F3USB hw_driver;
    MyUSBDeviceDriver device_driver((uint8_t *)&dev_desc, (uint8_t *)&config_desc, &hw_driver);

    //enable ports
    GPIO::enable(PA);
    GPIO::enable(PB);
    GPIO::enable(PC);
    GPIO::enable(PD);
    GPIO::enable(PE);

    // setup USB
    GPIO::setSpeed(PA11, _50MHz);
    GPIO::setSpeed(PA12, _50MHz);
    GPIO::setMode(PA11, AF);
    GPIO::setMode(PA12, AF);
    GPIO::setAF(PA11, AF14);
    GPIO::setAF(PA12, AF14);
    device_driver.init();

    /*** Setup TIM8 for PWM out ***/
    GPIO::setMode(PC6, AF);
    GPIO::setOutputType(PC6, PUSH_PULL);
    GPIO::setPullUpDown(PC6, NONE);
    GPIO::setSpeed(PC6, _10MHz);
    GPIO::setAF(PC6, AF4); // (TIM8_CH1)

    GPIO::setMode(PC7, AF);
    GPIO::setOutputType(PC7, PUSH_PULL);
    GPIO::setPullUpDown(PC7, NONE);
    GPIO::setSpeed(PC7, _10MHz);
    GPIO::setAF(PC7, AF4); // (TIM8_CH1)

    GPIO::setMode(PC8, AF);
    GPIO::setOutputType(PC8, PUSH_PULL);
    GPIO::setPullUpDown(PC8, NONE);
    GPIO::setSpeed(PC8, _10MHz);
    GPIO::setAF(PC8, AF4); // (TIM8_CH1)

    GPIO::setMode(PC9, AF);
    GPIO::setOutputType(PC9, PUSH_PULL);
    GPIO::setPullUpDown(PC9, NONE);
    GPIO::setSpeed(PC9, _10MHz);
    GPIO::setAF(PC9, AF4); // (TIM8_CH1)

    // enable TIM8 clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    asm volatile("dmb");

    // set TIM8 prescaler (72MHz / (2 + 1) = 24MHz)
    TIM8->PSC = 2;

    // set TIM8 auto-reload (24MHz / 64000 = 375Hz)
    TIM8->ARR = 64000;

    // PWM mode 1 on channel 1/2
    TIM8->CCMR1 = (0x6 << 4) | (0x6 << 12);

    // PWM mode 1 on channel 3/4
    TIM8->CCMR2 = (0x6 << 4) | (0x6 << 12);

    // compare register
    TIM8->CCR1 = 0;
    TIM8->CCR2 = 0;
    TIM8->CCR3 = 0;
    TIM8->CCR4 = 0;

    // enable channel1
    TIM8->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    // enable counter
    TIM8->CR1 |= TIM_CR1_CEN;

    // enable MOE and AOE
    TIM8->BDTR |= 1 << 15 | 1 << 14;

    /*** initialize timers for PWM input capture ***/
    reciever_input_init();

    /*** init I2C accel ***/
    GPIO::setSpeed(PB6, _2MHz);
    GPIO::setSpeed(PB7, _2MHz);
    GPIO::setOutputType(PB6, OPEN_DRAIN);
    GPIO::setOutputType(PB7, OPEN_DRAIN);
    GPIO::setPullUpDown(PB6, PULL_UP);
    GPIO::setPullUpDown(PB7, PULL_UP);
    GPIO::setMode(PB6, AF);
    GPIO::setMode(PB7, AF);
    GPIO::setAF(PB6, AF4);
    GPIO::setAF(PB7, AF4);

    // init accelerometer
    i2c_init(I2C1);
    i2c_set_addr(I2C1, LSM303DLHC_ACCEL_ADDR);

    i2c_start_write(I2C1, 2);
    i2c_write(I2C1, CTRL_REG1_A);
    i2c_write(I2C1, 0x77);

    i2c_start_write(I2C1, 2);
    i2c_write(I2C1, CTRL_REG4_A);
    i2c_write(I2C1, 0x08);

    i2c_stop(I2C1);


    /*********** SPI GYRO ***********/
    GPIO::setSpeed(PA5, _50MHz);
    GPIO::setSpeed(PA6, _50MHz);
    GPIO::setSpeed(PA7, _50MHz);

    GPIO::setSpeed(PE1, _50MHz);
    GPIO::setSpeed(PE3, _50MHz);

    GPIO::setMode(PA5, AF);
    GPIO::setMode(PA6, AF);
    GPIO::setMode(PA7, AF);

    GPIO::setMode(PE1, IN);
    GPIO::setMode(PE3, OUT);

    GPIO::setPullUpDown(PE1, PULL_DOWN);

    // setup gyro interrupt pin (PE1)
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // enable SYSCFG clock
    asm volatile("dmb");

    SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PE_gc; // EXTI1 triggered by PE1
    EXTI->IMR |= EXTI_IMR_MR1; // EXTI1 not masked
    EXTI->RTSR |= EXTI_RTSR_TR1; // rising-edge triggered
    //NVIC_EnableIRQ(EXTI1_IRQn);

    GPIO::setAF(PA5, AF5);
    GPIO::setAF(PA6, AF5);
    GPIO::setAF(PA7, AF5);

    // setup and enable SPI1
    // enable clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    asm volatile("dmb");

    SPI1->CR2 = (0xF << SPI_CR2_DS_gp); // 16-bit
    SPI1->CR1 =
        SPI_CR1_SPE // enable
        | SPI_CR1_MSTR // master
        | (0x1 << SPI_CR1_BR_gp) // 9MHz
        | SPI_CR1_CPOL_gm
        | SPI_CR1_CPHA_gm
        | SPI_CR1_SSM_gm // software slave management
        | SPI_CR1_SSI_gm // internal CS/SS high (to enable communication)
    ;

    // configure the gyro
    spi_rw16_single(SPI1, L3GD20_WRITE | L3GD20_CTRL_REG1 | 0xBF, PE3);
    spi_rw16_single(SPI1, L3GD20_WRITE | L3GD20_CTRL_REG2 | 0x29, PE3);
    spi_rw16_single(SPI1, L3GD20_WRITE | L3GD20_CTRL_REG3 | 0x08, PE3);
    spi_rw16_single(SPI1, L3GD20_WRITE | L3GD20_CTRL_REG4 | 0x20, PE3);
    spi_rw16_single(SPI1, L3GD20_WRITE | L3GD20_CTRL_REG5 | 0x12, PE3);


    // use PE9 to time how long this code takes to run
    // and for leveling

    // positive pitch
    GPIO::setMode(PE9, OUT);
    GPIO::setOutputType(PE9, PUSH_PULL);
    GPIO::setPullUpDown(PE9, NONE);
    GPIO::setSpeed(PE9, _10MHz);

    // negative pitch
    GPIO::setMode(PE13, OUT);
    GPIO::setOutputType(PE13, PUSH_PULL);
    GPIO::setPullUpDown(PE13, NONE);
    GPIO::setSpeed(PE13, _10MHz);

    // positive roll
    GPIO::setMode(PE15, OUT);
    GPIO::setOutputType(PE15, PUSH_PULL);
    GPIO::setPullUpDown(PE15, NONE);
    GPIO::setSpeed(PE15, _10MHz);

    // negative roll
    GPIO::setMode(PE11, OUT);
    GPIO::setOutputType(PE11, PUSH_PULL);
    GPIO::setPullUpDown(PE11, NONE);
    GPIO::setSpeed(PE11, _10MHz);

    // do a read so the next data ready interrupt is caught
    spi_rw16_single(SPI1, L3GD20_READ | L3GD20_OUT_X_H, PE3);

    for(;;)
    {
        uint16_t dr;
        uint16_t gyro_x = 0, gyro_y = 0, gyro_z = 0;

        if(EXTI->PR & EXTI_PR_PR1) // new gyro data is ready
        {
            g_counter = TIM2->CNT;

            EXTI->PR |= EXTI_PR_PR1;

            /*** read gyro ***/
            dr = spi_rw16_mul_begin(SPI1, L3GD20_READ | L3GD20_ADDR_INC_ON | L3GD20_OUT_X_L, PE3);
            gyro_x = dr & 0xFF;
            
            dr = spi_rw16_mul_next(SPI1, 0);
            gyro_x |= dr & 0xFF00;
            gyro_y = dr & 0xFF;

            dr = spi_rw16_mul_next(SPI1, 0);
            gyro_y |= dr & 0xFF00;
            gyro_z = dr & 0xFF;

            dr = spi_rw16_mul_end(SPI1, 0, PE3);
            gyro_z |= dr & 0xFF00;

            g_telemetry.gyro.x = -(*((int16_t *)&gyro_x));
            g_telemetry.gyro.y = -(*((int16_t *)&gyro_y));
            g_telemetry.gyro.z = (*((int16_t *)&gyro_z));
            g_telemetry.gyro *= 0.070f * (pi / 180.0f); // scale to radians/sec

            /*** read accel ***/
            uint8_t high, low;

            i2c_set_addr(I2C1, LSM303DLHC_ACCEL_ADDR);
            i2c_start_write(I2C1, 1);
            i2c_write(I2C1, OUT_X_L_A | (1 << 7));
            i2c_start_read(I2C1, 6);

            low = i2c_read(I2C1);
            high = i2c_read(I2C1);
            g_telemetry.accel.x = int16_t((high << 8) | low);

            low = i2c_read(I2C1);
            high = i2c_read(I2C1);
            g_telemetry.accel.y = int16_t((high << 8) | low);

            low = i2c_read(I2C1);
            high = i2c_read(I2C1);
            g_telemetry.accel.z = int16_t((high << 8) | low);

            i2c_stop(I2C1);

            g_telemetry.accel.normalize();


            /*** R/C receiver data ***/
            g_telemetry.rx_chan[0].period = (1.0 / 24e6f) * (TIM2->CCR1 & 0xFFFF);
            g_telemetry.rx_chan[0].pulse_width = (1.0 / 24e6f) * (TIM2->CCR2 & 0xFFFF);

            g_telemetry.rx_chan[1].period = (1.0 / 24e6f) * (TIM3->CCR1 & 0xFFFF);
            g_telemetry.rx_chan[1].pulse_width = (1.0 / 24e6f) * (TIM3->CCR2 & 0xFFFF);

            g_telemetry.rx_chan[2].period = (1.0 / 24e6f) * (TIM4->CCR1 & 0xFFFF);
            g_telemetry.rx_chan[2].pulse_width = (1.0 / 24e6f) * (TIM4->CCR2 & 0xFFFF);

            g_telemetry.rx_chan[3].period = (1.0 / 24e6f) * (TIM15->CCR1 & 0xFFFF);
            g_telemetry.rx_chan[3].pulse_width = (1.0 / 24e6f) * (TIM15->CCR2 & 0xFFFF);


            float pos_throttle = std::min(std::max(g_telemetry.rx_chan[0].pulse_width - g_settings.servo_pulse_min_in, 0.0f) / (g_settings.servo_pulse_max_in - g_settings.servo_pulse_min_in), 1.0f);
            float pos_pitch = std::min(std::max(g_telemetry.rx_chan[1].pulse_width - g_settings.servo_pulse_min_in, 0.0f) / (g_settings.servo_pulse_max_in - g_settings.servo_pulse_min_in), 1.0f);
            float pos_roll = std::min(std::max(g_telemetry.rx_chan[2].pulse_width - g_settings.servo_pulse_min_in, 0.0f) / (g_settings.servo_pulse_max_in - g_settings.servo_pulse_min_in), 1.0f);
            float pos_yaw = std::min(std::max(g_telemetry.rx_chan[3].pulse_width - g_settings.servo_pulse_min_in, 0.0f) / (g_settings.servo_pulse_max_in - g_settings.servo_pulse_min_in), 1.0f);

            // pitch/roll/yaw will be 0.5 centered, but should be 0 with a -1 to 1 range
            pos_pitch = (pos_pitch * 2) - 1.0f;
            pos_roll = (pos_roll * 2) - 1.0f;
            pos_yaw = (pos_yaw * 2) - 1.0f;

            float cmd_throttle = std::min(pos_throttle, g_settings.throttle_max);
            float cmd_pitch = pos_pitch * g_settings.pitch_range;
            float cmd_roll = pos_roll * g_settings.roll_range;
            float cmd_yaw = pos_yaw * g_settings.yaw_range;

            /*** process IMU data ***/

            // gyro_filtered is only used for zeroing the gyro when commanded over USB
            const float filt_c_yaw = 0.99f;
            g_flightctrl.gyro_filtered = (g_flightctrl.gyro_filtered * filt_c_yaw) + (g_telemetry.gyro * (1.0f - filt_c_yaw));

            // accel_filtered is only used for zeroing the accelerometer when commanded over USB
            const float filt_c_accel = 0.99f;
            g_flightctrl.accel_filtered = (g_flightctrl.accel_filtered * filt_c_accel) + (g_telemetry.accel * (1.0f - filt_c_accel));

            // calibrate gyro
            g_telemetry.gyro -= g_settings.gyro_zero;
            g_telemetry.gyro *= g_settings.accel_zero;

            // calibrate accelerometer
            g_telemetry.accel *= g_settings.accel_zero;

            // complementary filter using small angle approximation on gyro delta
            g_flightctrl.up = ((g_flightctrl.up + (g_telemetry.gyro * dt)) * g_settings.filt_c) + (g_telemetry.accel * (1.0f - g_settings.filt_c));
            g_flightctrl.up.normalize();

            // re-orient with new sensed up vector
            Vec3f rot_axis = UP_AXIS.cross(g_flightctrl.up);
            rot_axis.normalize();
            float rot_dot = UP_AXIS.dot(g_flightctrl.up);
            Quatf orientation(rot_axis, acosf(rot_dot));
            Vec3f right = RIGHT_AXIS * orientation;
            Vec3f forward = FWD_AXIS * orientation;

            // pitch up is positive
            volatile float sign = g_flightctrl.up.dot(FWD_AXIS);
            volatile float pitch_dot = std::min(std::max(forward.dot(FWD_AXIS), 0.0f), 1.0f);
            volatile float pitch = (sign > 0) ? acosf(pitch_dot) : -acosf(pitch_dot);

            // roll right is positive
            sign = g_flightctrl.up.dot(RIGHT_AXIS);
            volatile float roll_dot = std::min(std::max(right.dot(RIGHT_AXIS), 0.0f), 1.0f);
            volatile float roll = (sign > 0) ? acosf(roll_dot) : -acosf(roll_dot);

            // yaw
            float yaw = g_telemetry.gyro.z;

            g_telemetry.pitch = pitch;
            g_telemetry.roll = roll;
            g_telemetry.yaw = yaw;

            // light up the LEDs on the board so it can be used as a level
            if(pitch > 0)
            {
                GPIO::set(PE9);
                GPIO::clear(PE13);
            }
            else
            {
                GPIO::set(PE13);
                GPIO::clear(PE9);
            }

            if(roll > 0)
            {
                GPIO::set(PE15);
                GPIO::clear(PE11);
            }
            else
            {
                GPIO::set(PE11);
                GPIO::clear(PE15);
            }

            float pitch_correct = g_flightctrl.pitch_ctrl.control(cmd_pitch, pitch, dt);
            float roll_correct = g_flightctrl.roll_ctrl.control(cmd_roll, roll, dt);
            float yaw_correct = g_flightctrl.yaw_ctrl.control(cmd_yaw, g_telemetry.yaw, dt);

            pitch_correct = (cmd_throttle > g_settings.throttle_min_ctrl) ? pitch_correct : 0.0f;
            roll_correct = (cmd_throttle > g_settings.throttle_min_ctrl) ? roll_correct : 0.0f;
            yaw_correct = (cmd_throttle > g_settings.throttle_min_ctrl) ? yaw_correct : 0.0f;

            g_telemetry.cmd_throttle = cmd_throttle;
            g_telemetry.cmd_pitch = cmd_pitch;
            g_telemetry.cmd_roll = cmd_roll;
            g_telemetry.cmd_yaw = cmd_yaw;

            g_telemetry.pitch_correct = pitch_correct;
            g_telemetry.roll_correct = roll_correct;
            g_telemetry.yaw_correct = yaw_correct;

            // front left is CW and alternates in order fl, fr, rr, rl
            // value is power level 0.0f(0%) to 1.0f(100%)
            float fl = std::min(std::max(cmd_throttle + pitch_correct + roll_correct + yaw_correct, 0.0f), 1.0f);
            float fr = std::min(std::max(cmd_throttle + pitch_correct - roll_correct - yaw_correct, 0.0f), 1.0f);
            float rr = std::min(std::max(cmd_throttle - pitch_correct - roll_correct + yaw_correct, 0.0f), 1.0f);
            float rl = std::min(std::max(cmd_throttle - pitch_correct + roll_correct - yaw_correct, 0.0f), 1.0f);

            // now change to pulse widths in seconds
            fl = g_settings.servo_pulse_min_out + (fl * (g_settings.servo_pulse_max_out - g_settings.servo_pulse_min_out));
            fr = g_settings.servo_pulse_min_out + (fr * (g_settings.servo_pulse_max_out - g_settings.servo_pulse_min_out));
            rr = g_settings.servo_pulse_min_out + (rr * (g_settings.servo_pulse_max_out - g_settings.servo_pulse_min_out));
            rl = g_settings.servo_pulse_min_out + (rl * (g_settings.servo_pulse_max_out - g_settings.servo_pulse_min_out));

            //DONE! just set TIM8 CC regs, the PWM period is synchronized with right now
            // TIM8 is running at 24MHz

            // disable counter
            TIM8->CR1 &= ~TIM_CR1_CEN;

            // reset counter
            TIM8->EGR |= TIM_EGR_UG;

            // set compare values
            TIM8->CCR1 = (24e6f * fl);
            TIM8->CCR2 = (24e6f * fr);
            TIM8->CCR3 = (24e6f * rr);
            TIM8->CCR4 = (24e6f * rl);

            // re-enable counter to start new PWM period
            TIM8->CR1 |= TIM_CR1_CEN;

            device_driver.writeTelemetry();
            
            g_counter = TIM2->CNT - g_counter;
        }

        device_driver.process();
    }
}

void tim_pwm_capture_init(TIM_TypeDef *tim)
{
    tim->PSC = 2; // 72MHz/2+1 = 24MHz clock

    // capture channel 1 mapped to TI1, capture channel 2 mapped to TI1
    tim->CCMR1 = (0x1 << TIM_CCMR1_CC1S_gp) | (0x2 << TIM_CCMR1_CC2S_gp);

    // TI1FP1 rising edge trigger
    tim->CCER = (tim->CCER & ~(TIM_CCER_CC1P_gm | TIM_CCER_CC1NP_gm)) | (0x0 << TIM_CCER_CC1P_gp);
    // TI1FP2 falling edge trigger
    tim->CCER = (tim->CCER & ~(TIM_CCER_CC2P_gm | TIM_CCER_CC2NP_gm)) | (0x1 << TIM_CCER_CC2P_gp);

    // trigger input to TI1FP1
    tim->SMCR = (tim->SMCR & ~TIM_SMCR_TS_gm) | (0x5 << TIM_SMCR_TS_gp);
    // slave reset mode
    tim->SMCR = (tim->SMCR & ~TIM_SMCR_SMS_gm) | (0x0 << TIM_SMCR_SMS3_gp) | (0x4 << TIM_SMCR_SMS0_gp);

    // enable capture channels 1 and 2
    tim->CCER |= TIM_CCER_CC1E_gm | TIM_CCER_CC2E_gm;

    // generate update event (shadow shit)
    tim->EGR |= TIM_EGR_UG;

    // enable counter
    tim->CR1 |= TIM_CR1_CEN_gm;
}

void reciever_input_init()
{
    /*** TIM2 ***/
    // PA15 as TIM2_CH1
    //GPIO::enable(PA);
    //GPIO::setSpeed(PA15, _10MHz);
    //GPIO::setPullUpDown(PA15, PULL_DOWN);
    GPIO::setMode(PA15, AF);
    GPIO::setAF(PA15, AF1);

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    asm volatile("dmb");

    tim_pwm_capture_init(TIM2);
    

    /*** TIM3 ***/
    // PB4 as TIM3_CH1
    //GPIO::enable(PB);
    //GPIO::setSpeed(PB4, _50MHz);
    //GPIO::setPullUpDown(PB4, PULL_UP);
    GPIO::setMode(PB4, AF);
    GPIO::setAF(PB4, AF2);

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    asm volatile("dmb");

    tim_pwm_capture_init(TIM3);

    /*** TIM4 ***/
    // PD12 as TIM4_CH1
    //GPIO::enable(PD);
    //GPIO::setSpeed(PD12, _10MHz);
    //GPIO::setPullUpDown(PD12, PULL_UP);
    GPIO::setMode(PD12, AF);
    GPIO::setAF(PD12, AF2);

    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    asm volatile("dmb");

    tim_pwm_capture_init(TIM4);

    /*** TIM15 ***/
    // PA2 as TIM15_CH1
    //GPIO::enable(PA);
    //GPIO::setSpeed(PA2, _50MHz);
    //GPIO::setPullUpDown(PA2, PULL_UP);
    GPIO::setMode(PA2, AF);
    GPIO::setAF(PA2, AF9);

    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    asm volatile("dmb");

    tim_pwm_capture_init(TIM15);
}