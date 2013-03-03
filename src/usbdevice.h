#ifndef USBDEVICE_H
#define USBDEVICE_H

#include <cstdint>

#define IN_EP(ep) ((ep & 0x0F) | 0x80)
#define OUT_EP(ep) (ep & 0x0F)

namespace USB
{    
    struct SetupPacket
    {
        uint8_t bmRequestType;
        uint8_t bRequest;
        uint16_t wValue;
        uint16_t wIndex;
        uint16_t wLength;

        uint8_t getDirection() const { return bmRequestType & 0x80; }
        uint8_t getType() const { return bmRequestType & 0x60; }
        uint8_t getRecipient() const { return bmRequestType & 0x0F; }

        enum : uint8_t
        {
            // bRequest
            GET_STATUS = 0x00,
            CLEAR_FEATURE = 0x01,
            SET_FEATURE = 0x03,
            SET_ADDRESS = 0x05,
            GET_DESCRIPTOR = 0x06,
            SET_DESCRIPTOR = 0x07,
            GET_CONFIGURATION = 0x08,
            SET_CONFIGURATION = 0x09,

            // bmRequestType Direction bitmask, not shifted
            DATA_OUT = 0x00,
            DATA_IN = 0x80,

            // bmRequestType Type bitmask, not shifted
            STANDARD = 0x00,
            CLASS = 0x20,
            VENDOR = 0x40,

            // bmRequestType Recipient bitmask, not shifted
            DEVICE = 0x00,
            INTERFACE = 0x01,
            ENDPOINT = 0x02,
            OTHER = 0x03
        };
    };
    static_assert(sizeof(SetupPacket) == 8, "sizeof(USB::SetupPacket) should be 8");

    struct GenericDescriptor
    {
        const uint8_t * const data;

        GenericDescriptor() = delete;
        constexpr GenericDescriptor(const uint8_t * const desc) : data(desc) {}

        constexpr uint8_t getLength() const { return data[0]; }
        constexpr uint8_t getDescriptorType() const { return data[1]; }
    };

    struct DeviceDescriptor
    {
        const uint8_t * const data;

        DeviceDescriptor() = delete;
        constexpr DeviceDescriptor(const uint8_t * const desc) : data(desc) {}

        static constexpr uint8_t getLength() { return 18; }
        static constexpr uint8_t getDescriptorType() { return 1; }
        constexpr uint8_t getMaxPacketSize() const { return data[7]; }

        //TODO: add more functions for the rest of the data
    };

    struct EndpointDescriptor
    {
        const uint8_t * const data;

        enum : uint8_t
        {
            CONTROL = 0,
            ISO = 1,
            BULK = 2,
            INTERRUPT = 3
        };

        EndpointDescriptor() = delete;
        constexpr EndpointDescriptor(const uint8_t * const desc) : data(desc) {}

        static constexpr uint8_t getLength() { return 7; }
        static constexpr uint8_t getDescriptorType() { return 5; }
        constexpr uint8_t getAddress() const { return data[2]; }
        constexpr uint8_t getEndpointNumber() const { return data[2] & 0x0F; }
        constexpr uint8_t getDirection() const { return data[2] >> 7; }
        constexpr uint8_t getAttributes() const { return data[3]; }
        constexpr uint8_t getType() const { return data[3] & 0x03; }
        constexpr uint16_t getMaxPacketSize() const { return data[4] | (data[5] << 8); }
    };

    struct InterfaceDescriptor
    {
        const uint8_t * const data;

        InterfaceDescriptor() = delete;
        constexpr InterfaceDescriptor(const uint8_t * const desc) : data(desc) {}

        static constexpr uint8_t getLength() { return 9; }
        static constexpr uint8_t getDescriptorType() { return 4; }
        constexpr uint8_t getInterfaceNumber() const { return data[2]; }
        constexpr uint8_t getAlternateSetting() const { return data[3]; }
        constexpr uint8_t getNumEndpoints() const { return data[4]; }
        constexpr uint8_t getInterfaceClass() const { return data[5]; }
        constexpr uint8_t getInterfaceSubClass() const { return data[6]; }
        constexpr uint8_t getInterfaceProtocol() const { return data[7]; }
        constexpr uint8_t getStringIndex() const { return data[8]; }

        constexpr EndpointDescriptor getEndpointByAddress(uint8_t addr) const { return getEndpointByAddress(addr, GenericDescriptor(data + getLength())); }
        constexpr EndpointDescriptor getEndpointByIdx(int i) const { return getEndpointByIdx(i, 0, GenericDescriptor(data + getLength())); }

    private:
        constexpr EndpointDescriptor getEndpointByAddress(uint8_t addr, GenericDescriptor desc) const
        {
            return (desc.getDescriptorType() == EndpointDescriptor::getDescriptorType() && EndpointDescriptor(desc.data).getAddress() == addr) ? EndpointDescriptor(desc.data) :
                getEndpointByAddress(
                    addr,
                    GenericDescriptor(
                        desc.data + desc.getLength()
                    )
                )
            ;
        }

        constexpr EndpointDescriptor getEndpointByIdx(int i, int idx, GenericDescriptor desc) const
        {
            return (i == idx && desc.getDescriptorType() == EndpointDescriptor::getDescriptorType()) ? EndpointDescriptor(desc.data) :
                getEndpointByIdx(
                    i,
                    (desc.getDescriptorType() == EndpointDescriptor::getDescriptorType()) ? idx + 1 : idx,
                    GenericDescriptor(
                        desc.data + desc.getLength()
                    )
                )
            ;
        }
    };

    struct ConfigurationDescriptor
    {
        const uint8_t * const data;

        ConfigurationDescriptor() = delete;
        constexpr ConfigurationDescriptor(const uint8_t * const desc) : data(desc) {}

        static constexpr uint8_t getLength() { return 9; }
        static constexpr uint8_t getDescriptorType() { return 2; }
        constexpr uint16_t getTotalLength() const { return data[2] | (data[3] << 8); }
        constexpr uint8_t getNumInterfaces() const { return data[4]; }
        constexpr uint8_t getConfigurationValue() const { return data[5]; }
        constexpr uint8_t getStringIndex() const { return data[6]; }
        constexpr uint8_t getAttributes() const { return data[7]; }
        constexpr uint8_t getMaxPower() const { return data[8]; }

        constexpr InterfaceDescriptor getInterfaceByNum(int i) const { return getInterfaceByNum(i, GenericDescriptor(data + getLength())); }
        constexpr InterfaceDescriptor getInterfaceByIdx(int i) const { return getInterfaceByIdx(i, 0, GenericDescriptor(data + getLength())); }

    private:
        constexpr InterfaceDescriptor getInterfaceByNum(int i, GenericDescriptor desc) const
        {
            return (desc.getDescriptorType() == InterfaceDescriptor::getDescriptorType() && InterfaceDescriptor(desc.data).getInterfaceNumber() == i) ? InterfaceDescriptor(desc.data) :
                getInterfaceByNum(
                    i,
                    GenericDescriptor(
                        desc.data + desc.getLength()
                    )
                )
            ;
        }

        constexpr InterfaceDescriptor getInterfaceByIdx(int i, int idx, GenericDescriptor desc) const
        {
            return (i == idx && desc.getDescriptorType() == InterfaceDescriptor::getDescriptorType()) ? InterfaceDescriptor(desc.data) :
                getInterfaceByIdx(
                    i,
                    (desc.getDescriptorType() == InterfaceDescriptor::getDescriptorType()) ? idx + 1 : idx,
                    GenericDescriptor(
                        desc.data + desc.getLength()
                    )
                )
            ;
        }
    };

    class USBDeviceDriver;

    class USBHardwareDriver
    {
    public:
        virtual void init(USBDeviceDriver *usb_device) = 0;
        virtual void postInit() = 0;
        virtual void reset() = 0;
        virtual bool initEndpoint(uint8_t ep, uint16_t max_packet_size, uint8_t ep_type) = 0;
        virtual void process() = 0;
        virtual void write(uint8_t ep, const uint8_t *buf, uint32_t len) = 0;
        virtual uint32_t read(uint8_t ep, uint8_t * buf, uint32_t len) = 0;
        virtual void setOutTokenHandshake(uint8_t ep, int handshake) = 0;
        virtual void setInTokenHandshake(uint8_t ep, int handshake) = 0;
    };

    class USBDeviceDriver
    {
    private:
        DeviceDescriptor device_desc;
        ConfigurationDescriptor config_desc;
        USBHardwareDriver *hw_driver;
        bool already_init = false;


    public:
        enum
        {
            STALL = 1,
            NAK = 2,
            ACK = 3
        };

        USBDeviceDriver(const uint8_t * const d_desc, const uint8_t * const c_desc, USBHardwareDriver *hw_drv) : device_desc(d_desc), config_desc(c_desc), hw_driver(hw_drv) {}

        const DeviceDescriptor &getDeviceDescriptor() { return device_desc; }
        const ConfigurationDescriptor &getConfigurationDescriptor() { return config_desc; }

        virtual void init()
        {
            hw_driver->init(this);
        }

        void reset()
        {
            hw_driver->reset();
        }

        virtual void process()
        {
            hw_driver->process();
        }

        void write(uint8_t ep, const uint8_t *buf, uint32_t len)
        {
            hw_driver->write(ep, buf, len);
        }

        uint32_t read(uint8_t ep, uint8_t *buf, uint32_t len)
        {
            return hw_driver->read(ep, buf, len);
        }

        void setOutTokenHandshake(uint8_t ep, int handshake)
        {
            hw_driver->setOutTokenHandshake(ep, handshake);
        }

        void setInTokenHandshake(uint8_t ep, int handshake)
        {
            hw_driver->setInTokenHandshake(ep, handshake);
        }

        /*** functions called by the hardware driver ***/

        virtual void onReset()
        {
            for(int i = 0; i < config_desc.getNumInterfaces(); ++i)
            {
                InterfaceDescriptor interface = config_desc.getInterfaceByIdx(i);

                for(int j = 0; j < interface.getNumEndpoints(); ++j)
                {
                    EndpointDescriptor ep = interface.getEndpointByIdx(j);

                    hw_driver->initEndpoint(
                        ep.getAddress(),
                        ep.getMaxPacketSize(),
                        ep.getType()
                    );
                }
            }

            hw_driver->postInit();
        }

        virtual void onWriteComplete(uint8_t ep) = 0;
        virtual void onReadReady(uint8_t ep, uint32_t len) = 0;
        virtual void onSetupRequest(uint8_t ep, const SetupPacket &setup) = 0;
    };
}

#endif
