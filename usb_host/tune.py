#!/usr/bin/env python2

import usb.core
import usb.util
import struct
import math
import sys


dev = usb.core.find(idVendor=0xDEAD, idProduct=0xBEEF)

if dev is None:
    raise ValueError('Device not found')

"""
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

    float pitch_range = pi / 12.0f; // +/-45 deg
    float roll_range = pi / 12.0f; // +/-45 deg
    float yaw_range = pi * 2.0f; // +/- 360deg/s

    float throttle_max = 0.9f;
    float throttle_min_ctrl = 0.1; // won't PID control below this

    float esc_trim[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    PIDProperties pitch_prop = {0.5f, 0.0f, 0.0f, 0.0f};
    PIDProperties roll_prop = {0.5f, 0.0f, 0.0f, 0.0f};
    PIDProperties yaw_prop = {0.05f, 0.0f, 0.0f, 0.0f};

    Vec3f gyro_zero;
    Quatf accel_zero;
}
"""

#write settings
settings_struct = (
    0x00000007, # zero gyro and accel
	0.995,

	0.001,
	0.002,

	0.001,
	0.002,

	math.pi / 4.0,
	math.pi / 4.0,
	math.pi * 2,

	0.9,
	0.1,

    0.0, 0.0, 0.0, 0.0,

	0.0625, 0.0001, 0.0, 0.0,
	0.0625, 0.0001, 0.0, 0.0,
	0.00625, 0.0, 0.0, 0.0,

    0.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 1.0
)

settings = struct.pack('<1L33f', *settings_struct)
dev.write(0x1, settings, 0, timeout=100)

data = dev.read(0x81, len(settings), 0, timeout=100)
print('data len: ' + str(len(data)))

floats = struct.unpack('<1L33f', data)
for val in floats:
	print(val)