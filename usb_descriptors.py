def usbDeviceDescriptor(bcdUSB=0, bDeviceClass=0, bDeviceSubClass=0, bDeviceProtocol=0, bMaxPacketSize=0, idVendor=0, idProduct=0, bcdDevice=0, iManufacturer=0, iProduct=0, iSerialNumber=0, bNumConfigurations=0):
    bytes = [0] * 0x12
    result = ''

    bytes[0] = 0x12
    bytes[1] = 0x01
    bytes[2] = bcdUSB & 0xFF
    bytes[3] = (bcdUSB >> 8) & 0xFF
    bytes[4] = bDeviceClass
    bytes[5] = bDeviceSubClass
    bytes[6] = bDeviceProtocol
    bytes[7] = bMaxPacketSize
    bytes[8] = idVendor
    bytes[9] = idProduct
    bytes[10] = bcdDevice & 0xFF
    bytes[11] = (bcdDevice >> 8) & 0xFF
    bytes[12] = iManufacturer
    bytes[13] = iProduct
    bytes[14] = iSerialNumber
    bytes[15] = bNumConfigurations

    for i, byte in enumerate(bytes):
        result += '0x%02X' % (byte)

        if i != len(bytes) - 1:
            result += ', '

    return result

def usbEndpointDescriptor(bEndpointAddress, bmAttributes, wMaxPacketSize, bInterval):
    bytes = [0] * 7

    bytes[0] = 0x07
    bytes[1] = 0x05
    bytes[2] = bEndpointAddress
    bytes[3] = bmAttributes
    bytes[4] = wMaxPacketSize & 0xFF
    bytes[5] = (wMaxPacketSize >> 8) & 0xFF
    bytes[6] = bInterval

    return bytes

def usbInterfaceDescriptor(bInterfaceNumber=0, bAlternateSetting=0, bInterfaceClass=0, bInterfaceSubClass=0, bInterfaceProtocol=0, iInterface=0, endpoints=None):
    bytes = [0] * 9

    bytes[0] = 0x09
    bytes[1] = 0x04
    bytes[2] = bInterfaceNumber
    bytes[3] = bAlternateSetting
    bytes[4] = len(endpoints)
    bytes[5] = bInterfaceClass
    bytes[6] = bInterfaceSubClass
    bytes[7] = bInterfaceProtocol
    bytes[8] = iInterface

    for ep in endpoints:
        bytes.extend(ep)

    return bytes

def usbConfigurationDescriptor(bConfigurationValue=0, iConfiguration=0, bmAttributes=0, bMaxPower=0, interfaces=None):
    bytes = [0] * 9
    total_size = 9
    result = ''

    bytes[0] = 0x09
    bytes[1] = 0x02
    #bytes[2]
    #bytes[3]
    bytes[4] = len(interfaces)
    bytes[5] = bConfigurationValue
    bytes[6] = iConfiguration
    bytes[7] = bmAttributes
    bytes[8] = bMaxPower

    for iface in interfaces:
        total_size += len(iface)
        bytes.extend(iface)

    bytes[2] = total_size & 0xFF
    bytes[3] = (total_size >> 8) & 0xFF

    for i, byte in enumerate(bytes):
        result += '0x%02X' % (byte)

        if i != len(bytes) - 1:
            result += ', '

    return result