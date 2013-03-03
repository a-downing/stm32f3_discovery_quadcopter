from usb_descriptors import *

def gpio_pins(ports, pins):
    result = ''

    for port in ports:
        for pin in pins:
            result += 'P%s%d=0x%04X, ' % (port, pin, ((ord(port) - ord('A')) << 8) | pin)

        result += '\n'

    return result