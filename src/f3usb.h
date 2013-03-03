#ifndef F3USB_H
#define F3USB_H

#include <cstdint>

#include "usbdevice.h"
#include "f3usb_defs.h"

//#include "rblog.h"

using namespace USB;

class F3USB;
using F1USB = F3USB;

//RBLog<500, 4> Log;

class F3USB : public USB::USBHardwareDriver
{
private:
    struct EnumerationState
    {
        bool enumerated = false;

        uint8_t set_address = 0;
        int8_t set_configuration = -1;

        uint16_t dev_desc_size_requested = 0;
        uint16_t dev_desc_size_sent = 0;
        uint16_t dev_desc_max_tx = 0;
        uint16_t dev_desc_size_limbo = 0;

        uint16_t conf_desc_size_requested = 0;
        uint16_t conf_desc_size_sent = 0;
        uint16_t conf_desc_max_tx = 0;
        uint16_t conf_desc_size_limbo = 0;

        void dev_desc_reset()
        {
            dev_desc_size_requested = 0;
            dev_desc_size_sent = 0;
            dev_desc_max_tx = 0;
            dev_desc_size_limbo = 0;
        }

        void conf_desc_reset()
        {
            conf_desc_size_requested = 0;
            conf_desc_size_sent = 0;
            conf_desc_max_tx = 0;
            conf_desc_size_limbo = 0;
        }

        void reset()
        {
            enumerated = false;
            set_address = 0;
            set_configuration = -1;
            dev_desc_reset();
            conf_desc_reset();
        }
    };

    USB::USBDeviceDriver *device_driver = nullptr;
    EnumerationState e_state;

    USB::Registers &usb = *reinterpret_cast<USB::Registers * const>(USB::PERIPHERAL_ADDR);
    USB::BufferTable * const btable = reinterpret_cast<USB::BufferTable * const>(USB::SRAM_ADDR);
    uint32_t * const usb_sram = reinterpret_cast<uint32_t * const>(USB::SRAM_ADDR);
    uint32_t pkt_buf_offset = USB::BTABLE_END_OFFSET;

public:
    void handleCtr(int ep)
    {
        if(usb.EPR[ep] & USB::EPR::CTR_TX_bm)
        {
            //Log.log("got CTR_TX: ep=%d", ep);

            if(ep == 0 && e_state.enumerated == false)
            {
                // handle GET_DESCRIPTOR (device descriptor)
                if(e_state.dev_desc_size_requested != 0)
                {
                    if(e_state.dev_desc_size_sent <= e_state.dev_desc_size_requested)
                    {
                        //Log.log("    got CTR_TX for device descriptor transmission");

                        e_state.dev_desc_size_sent += e_state.dev_desc_size_limbo;
                        uint32_t remaining = e_state.dev_desc_size_requested - e_state.dev_desc_size_sent;
                        e_state.dev_desc_size_limbo = (remaining < e_state.dev_desc_max_tx) ? remaining : e_state.dev_desc_max_tx;

                        if(e_state.dev_desc_size_sent < e_state.dev_desc_size_requested)
                        {
                            // still sending the device descriptor
                            //Log.log("        sending remaining device descriptor");

                            write(
                                ep,
                                device_driver->getDeviceDescriptor().data + e_state.dev_desc_size_sent,
                                e_state.dev_desc_size_limbo
                            );
                        }
                        else
                        {
                            //Log.log("        device descriptor transmission complete");
                            e_state.dev_desc_reset();
                        }
                    }
                }

                // handle GET_DESCRIPTOR (configuration descriptor)
                if(e_state.conf_desc_size_requested != 0)
                {
                    if(e_state.conf_desc_size_sent <= e_state.conf_desc_size_requested)
                    {
                        //Log.log("    got CTR_TX for configuration descriptor transmission");

                        e_state.conf_desc_size_sent += e_state.conf_desc_size_limbo;
                        uint32_t remaining = e_state.conf_desc_size_requested - e_state.conf_desc_size_sent;
                        e_state.conf_desc_size_limbo = (remaining < e_state.conf_desc_max_tx) ? remaining : e_state.conf_desc_max_tx;

                        if(e_state.conf_desc_size_sent < e_state.conf_desc_size_requested)
                        {
                            // still sending the descriptor
                            //Log.log("        sending remaining configuration descriptor");

                            write(
                                ep,
                                device_driver->getConfigurationDescriptor().data + e_state.conf_desc_size_sent,
                                e_state.conf_desc_size_limbo
                            );
                        }
                        else
                        {
                            //Log.log("        configuration descriptor transmission complete");
                            e_state.conf_desc_reset();
                        }
                    }
                }

                // handle SET_ADDRESS
                if(e_state.set_address != 0)
                {
                    //Log.log("    got CTR_TX for SET_ADDRESS ZLP response");
                    //Log.log("        setting device address to %d", e_state.set_address);
                    setAddress(e_state.set_address);
                    e_state.set_address = 0;
                }

                // handle SET_CONFIGURATION
                if(e_state.set_configuration != -1)
                {
                    //Log.log("    got CTR_TX for SET_CONFIGURATION ZLP response");

                    e_state.set_configuration = -1;
                }
            }
            else
            {
                device_driver->onWriteComplete(ep | 0x80);
            }

            clearCtrTx(ep);
            //Log.log("cleared CTR_TX");
        }

        if(usb.EPR[ep] & USB::EPR::CTR_RX_bm)
        {
            ////Log.log("got CTR_RX (ep=%d, size=%d)", ep, btable[ep].COUNT_RX & USB::COUNT_RX::COUNT_bm);

            if(usb.EPR[ep] & USB::EPR::SETUP_bm)
            {
                // SETUP packet

                USB::SetupPacket setup;
                read(ep, reinterpret_cast<uint8_t *>(&setup), sizeof(setup));
                setRxStatus(ep, USB::EPR::STAT_RX::VALID);

                if(setup.getType() == USB::SetupPacket::STANDARD)
                {
                    handleSetupPacket(ep, setup);
                }
                else
                {
                    device_driver->onSetupRequest(ep, setup);
                }
            }
            else
            {
                if(ep == 0)
                {
                    if(getReadSize(ep) == 0)
                    {
                        //Log.log("    got a ZLP");
                        read(ep, nullptr, 0);
                    }
                }
                else
                {
                    device_driver->onReadReady(ep & 0x0F, getReadSize(ep));
                }
            }

            clearCtrRx(ep);
            //Log.log("cleared CTR_RX");
        }
    }

    void handleSetupPacket(int ep, const USB::SetupPacket &packet)
    {
        //Log.log("    got a SETUP packet: bmRequestType=0x%02X, bRequest=0x%02X, dir=0x%02X", packet.bmRequestType, packet.bRequest, packet.getDirection());

        if(packet.getDirection() == USB::SetupPacket::DATA_OUT)
        {
            if(packet.bRequest == USB::SetupPacket::SET_ADDRESS)
            {
                //Log.log("    got SET_ADDRESS");
                e_state.set_address = packet.wValue;

                write(ep, nullptr, 0);
                //Log.log("        writing SET_ADDRESS ZLP");
                return;
            }
            else if(packet.bRequest == USB::SetupPacket::SET_CONFIGURATION)
            {
                //Log.log("    got SET_CONFIGURATION");
                e_state.set_configuration = packet.wValue;

                write(ep, nullptr, 0);
                //Log.log("        writing SET_CONFIGURATION ZLP");
                return;
            }
        }
        else if(packet.getDirection() == USB::SetupPacket::DATA_IN)
        {
            if(packet.bRequest == USB::SetupPacket::GET_DESCRIPTOR)
            {
                //Log.log("    got GET_DESCRIPTOR, wValue=%d, wLength=%d", packet.wValue, packet.wLength);

                if(packet.wValue == 0x0100)
                {
                    // handle GET_DESCRIPTOR for device descriptor

                    //Log.log("        got device descriptor request");
                    
                    uint8_t max_packet_size = device_driver->getDeviceDescriptor().getMaxPacketSize();

                    e_state.dev_desc_size_requested = (device_driver->getDeviceDescriptor().getLength() < packet.wLength) ? device_driver->getDeviceDescriptor().getLength() : packet.wLength;

                    e_state.dev_desc_size_sent = 0;
                    e_state.dev_desc_max_tx = (e_state.dev_desc_size_requested < max_packet_size) ? e_state.dev_desc_size_requested : max_packet_size;
                    e_state.dev_desc_size_limbo = e_state.dev_desc_max_tx;

                    write(ep, device_driver->getDeviceDescriptor().data, e_state.dev_desc_max_tx);
                    //Log.log("        writing device descriptor to packet buffer");
                    return;
                }
                else if(packet.wValue == 0x0200)
                {
                    // handle GET_DESCRIPTOR for configuration descriptor

                    //Log.log("    got configuration descriptor request");

                    uint8_t max_packet_size = device_driver->getDeviceDescriptor().getMaxPacketSize();

                    e_state.conf_desc_size_requested = (device_driver->getConfigurationDescriptor().getTotalLength() < packet.wLength) ? device_driver->getConfigurationDescriptor().getTotalLength() : packet.wLength;
                    e_state.conf_desc_size_sent = 0;
                    e_state.conf_desc_max_tx = (e_state.conf_desc_size_requested < max_packet_size) ? e_state.conf_desc_size_requested : max_packet_size;
                    e_state.conf_desc_size_limbo = e_state.conf_desc_max_tx;

                    //Log.log("        writing configuration descriptor to packet buffer");
                    write(ep, device_driver->getConfigurationDescriptor().data, e_state.conf_desc_max_tx);
                    return;
                }
            }
        }

        //Log.log("    got UNKNOWN request");
        setTxStatus(ep, USB::EPR::STAT_TX::STALL);
        //Log.log("        setting STAT_TX to STALL");
    }

    void setTxStatus(int ep, uint32_t status)
    {
        using namespace USB::EPR;

        // this is pretty strange because of the behavior of the EPR register
        usb.EPR[ep] =
            ((usb.EPR[ep] & (EP_TYPE_bm | EP_KIND_bm | EA_bm | STAT_TX_bm)) ^ (status << STAT_TX_bp))
            | CTR_TX_bm
            | CTR_RX_bm
        ;
    }

    void setRxStatus(int ep, uint32_t status)
    {
        using namespace USB::EPR;

        usb.EPR[ep] =
            ((usb.EPR[ep] & (EP_TYPE_bm | EP_KIND_bm | EA_bm | STAT_RX_bm)) ^ (status << STAT_RX_bp))
            | CTR_TX_bm
            | CTR_RX_bm
        ;
    }

    uint32_t getTxStatus(int ep)
    {
        return (usb.EPR[ep] & USB::EPR::STAT_TX_bm) >> USB::EPR::STAT_TX_bp;
    }

    uint32_t getRxStatus(int ep)
    {
        return (usb.EPR[ep] & USB::EPR::STAT_RX_bm) >> USB::EPR::STAT_RX_bp;
    }

    void clearCtrTx(int ep)
    {
        using namespace USB::EPR;

        usb.EPR[ep] =
              (usb.EPR[ep] & (EP_TYPE_bm | EP_KIND_bm | EA_bm))
            | CTR_RX_bm
        ;
    }

    void clearCtrRx(int ep)
    {
        using namespace USB::EPR;

        usb.EPR[ep] =
              (usb.EPR[ep] & (EP_TYPE_bm | EP_KIND_bm | EA_bm))
            | CTR_TX_bm
        ;
    }

    void setAddress(uint8_t addr)
    {
        usb.DADDR = (usb.DADDR & (USB::DADDR::EF_bm | ~USB::DADDR::ADDR_bm)) | addr;
    }

    uint8_t getAddress(uint8_t addr)
    {
        return usb.DADDR & ~USB::DADDR::ADDR_bm;
    }

    uint32_t getReadSize(int ep)
    {
        return btable[ep].COUNT_RX & USB::COUNT_RX::COUNT_bm;
    }

    void powerOn()
    {
        usb.CNTR &= ~USB::CNTR::PDWN_bm;

        int cnt = 72; //at least 1us (more like 10)
        while(--cnt) { asm volatile("nop"); }
    }

    void powerOff()
    {
        usb.CNTR |= USB::CNTR::PDWN_bm;
    }

    void unReset()
    {
        usb.CNTR &= ~USB::CNTR::FRES_bm;
    }

    void enable()
    {
        usb.DADDR |= USB::DADDR::EF_bm;
    }

    void disable()
    {
        usb.DADDR &= ~USB::DADDR::EF_bm;
    }

    virtual void reset() override
    {
        usb.CNTR |= USB::CNTR::FRES_bm;
    }

    virtual void init(USB::USBDeviceDriver *dev) override
    {
        RCC->APB1ENR |= RCC_APB1ENR_USBEN_gm;
        asm volatile("dmb");

        powerOn();
        unReset();
        
        device_driver = dev;
    }

    virtual void postInit() override
    {
        asm volatile("nop");
    }

    virtual void setOutTokenHandshake(uint8_t ep, int handshake) override
    {
        setRxStatus(ep & 0x0F, handshake);
    }

    virtual void setInTokenHandshake(uint8_t ep, int handshake) override
    {
        setTxStatus(ep & 0x0F, handshake);
    }

    virtual void write(uint8_t ep, const uint8_t *buf, uint32_t len) override
    {
        ep &= 0x0F;

        uint32_t written = 0;
        uint32_t *sram = usb_sram + (btable[ep].ADDR_TX >> 1);

        while(written < len)
        {
            *sram = *buf++;
            ++written;

            if(written < len)
            {
                *sram |= (*buf++) << 8;
                ++written;
            }

            ++sram;
        }

        btable[ep].COUNT_TX = len;
        setTxStatus(ep, USB::EPR::STAT_TX::VALID);
    }

    virtual uint32_t read(uint8_t ep, uint8_t *buf, uint32_t len) override
    {
        ep &= 0x0F;

        uint32_t size = btable[ep].COUNT_RX & USB::COUNT_RX::COUNT_bm;
        size = (len < size) ? len : size;
        uint32_t read_total = 0;
        uint32_t *sram = usb_sram + (btable[ep].ADDR_RX >> 1);

        while(read_total < size)
        {
            *buf++ = *sram & 0xFF;
            ++read_total;

            if(read_total < size)
            {
                *buf++ = (*sram >> 8) & 0xFF;
                ++read_total;
            }

            ++sram;
        }

        btable[ep].COUNT_RX &= ~USB::COUNT_RX::COUNT_bm;
        //setRxStatus(ep, USB::EPR::STAT_RX::VALID);

        return read_total;
    }

    virtual void process() override
    {
        if(usb.ISTR & USB::ISTR::RESET_bm)
        {
            enable();

            pkt_buf_offset = USB::BTABLE_END_OFFSET;
            e_state.reset();
            setAddress(0);

            initEndpoint(0x00, 64, EndpointDescriptor::CONTROL);
            initEndpoint(0x80, 64, EndpointDescriptor::CONTROL);

            //GPIOE->ODR |= GPIO_ODR_13;
            usb.ISTR &= ~USB::ISTR::RESET_bm;

            device_driver->onReset();

            //Log.log("got RESET");
        }

        if(usb.ISTR & USB::ISTR::CTR_bm)
        {
            //Log.log("got CTR 0x%04X", usb.ISTR);
            handleCtr(usb.ISTR & USB::ISTR::EP_ID_bm);
        }
    }

    virtual bool initEndpoint(uint8_t ep, uint16_t max_packet_size, uint8_t ep_type) override
    {
        uint8_t dir = ((ep & 0x80) != 0);
        ep &= 0x0F;

        using namespace USB::COUNT_RX;
        using namespace USB::COUNT_TX;
        using namespace USB::EPR;

        uint32_t ep_kind = 0;

        if(ep_type == EndpointDescriptor::CONTROL)
        {
            ep_type = USB::EPR::EP_TYPE::CONTROL;
        }
        else if(ep_type == EndpointDescriptor::ISO)
        {
            ep_type = USB::EPR::EP_TYPE::ISO;
        }
        else if(ep_type == EndpointDescriptor::BULK)
        {
            ep_type = USB::EPR::EP_TYPE::BULK;
        }
        else if(ep_type == EndpointDescriptor::INTERRUPT)
        {
            ep_type = USB::EPR::EP_TYPE::INTERRUPT;
        }

        if(dir == 0) // OUT
        {
            // calculate BL_SIZE and NUM_BLOCK

            uint32_t bl_size;
            uint32_t num_block;
            if(max_packet_size % 2 == 0 && max_packet_size >= 2 && max_packet_size <= 62)
            {
                bl_size = BL_SIZE::BS_2_BYTES;
                num_block = max_packet_size / 2;
            }
            else if(max_packet_size % 32 == 0 && max_packet_size >= 32 && max_packet_size <= 512)
            {
                bl_size = BL_SIZE::BS_32_BYTES;
                num_block = (max_packet_size / 32) - 1;
            }
            else // unsupported buffer size
            {
                return false;
            }

            btable[ep].ADDR_RX = pkt_buf_offset;
                btable[ep].COUNT_RX =
                (bl_size << BL_SIZE_bp) |
                (num_block << NUM_BLOCK_bp)
            ;

            pkt_buf_offset += max_packet_size;
        }
        else if(dir == 1) // IN
        {
            btable[ep].ADDR_TX = pkt_buf_offset;
            btable[ep].COUNT_TX = 0;

            pkt_buf_offset += max_packet_size;
        }

        uint32_t epr = usb.EPR[ep];

        // set DTOG_TX/DTOG_RX to 0(DATA0) (probably don't need to)
        // set endpoint type and kind
        // toggle the stat_tx/stat_rx bits to NAK/VALID
        // CTR_TX/CTR_RX need to be written with invariant value(1) to not change current value
        // set endpoint address
        epr =
            /*  (epr & DTOG_TX_bm)
            | (epr & DTOG_RX_bm)
            |*/ (ep_type << EP_TYPE_bp)
            | (ep_kind << EP_KIND_bp)
            | ((epr & STAT_TX_bm) ^ (STAT_TX::NAK << STAT_TX_bp))
            | ((epr & STAT_RX_bm) ^ (STAT_RX::VALID << STAT_RX_bp))
            | CTR_TX_bm
            | CTR_RX_bm
            | (ep) // << EA_bp)
        ;

        usb.EPR[ep] = epr;

        return true;
    }
};

#endif
