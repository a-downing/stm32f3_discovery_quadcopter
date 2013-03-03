#!/usr/bin/env python2

import usb.core
import usb.util
import sys
import os
import struct
import math

class Angles:
	def __init__(self, pitch, roll, yaw):
		self.pitch = float(pitch)
		self.roll = float(roll)
		self.yaw = float(yaw)

	def toRadians(self):
		self.pitch = self.pitch * (math.pi / 180)
		self.roll = self.roll * (math.pi / 180)
		self.yaw = self.yaw * (math.pi / 180)

class Vector3:
	def __init__(self, x, y, z):
		self.x = float(x)
		self.y = float(y)
		self.z = float(z)

	def len(self):
		return math.sqrt(self.x**2 + self.y**2 + self.z**2);

	def len2(self):
		return self.x**2 + self.y**2 + self.z**2

	def normalize(self):
	    length = self.len()
	    self.x /= length
	    self.y /= length
	    self.z /= length

	def normalized(self):
	    length = self.len()
	    return Vector3(self.x / length, self.y / length, self.z / length)

	def smallAngApprox(self):
		norm = self.normalized()
		return Angles(norm.x, norm.y, norm.z)

	def angles(self):
		norm = self.normalized()
		return Angles(math.atan(norm.x), math.atan(norm.y), math.atan(norm.z))



dev = usb.core.find(idVendor=0xDEAD, idProduct=0xBEEF)

if dev is None:
    raise ValueError('Device not found')

"""
struct SensorData
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
"""

# print out R/C reciever pulse widths
"""
while True:
	data = dev.read(0x82, 96, 0, timeout=1000)
	floats = struct.unpack('24f', data)

	pitch = floats[6]
	roll = floats[7]
	yaw = floats[8]

	pitch_correct = floats[13]
	roll_correct = floats[14]
	yaw_correct = floats[15]

	ch1_period = floats[16]
	ch1_pulse_width = floats[17]

	ch2_period = floats[18]
	ch2_pulse_width = floats[19]

	ch3_period = floats[20]
	ch3_pulse_width = floats[21]

	ch4_period = floats[22]
	ch4_pulse_width = floats[23]

	print(str(pitch) + ' ' + str(pitch_correct))

	#os.system('clear')
	#print('ch1: ' + str(ch1_period) + '\t\t' + str(ch1_pulse_width))
	#print('ch2: ' + str(ch2_period) + '\t\t' + str(ch2_pulse_width))
	#print('ch3: ' + str(ch3_period) + '\t\t' + str(ch3_pulse_width))
	#print('ch4: ' + str(ch4_period) + '\t\t' + str(ch4_pulse_width))
"""

# pitch
#"""
angles = Angles(0, 0, 0)
angles2 = Angles(0, 0, 0)
estimate = Angles(0, 0, 0)
while True:
	data = dev.read(0x82, 96, 0, timeout=1000)
	floats = struct.unpack('24f', data)

	dt = 1.0 / 380.0
	c = 0.995

	gyro_delta = Angles(floats[0], floats[1], floats[2])
	accel_vec = Vector3(floats[3], floats[4], floats[5])
	accel_angles = accel_vec.angles()

	angles2.pitch = (angles2.pitch + gyro_delta.pitch * dt)

	angles.pitch = ((angles.pitch + (gyro_delta.pitch * dt)) * c) + (accel_angles.pitch * (1 - c))

	pitch = floats[6]

	print(str(gyro_delta.pitch) + ' ' + str(accel_angles.pitch) + ' ' + str(angles2.pitch) + ' ' + str(pitch))
#"""

# roll
angles = Angles(0, 0, 0)
angles2 = Angles(0, 0, 0)
estimate = Angles(0, 0, 0)
while True:
	data = dev.read(0x82, 64, 0, timeout=3)
	floats = struct.unpack('<16f', data)

	dt = 1.0 / 380.0
	c = 0.99

	gyro_delta = Angles(floats[0], floats[1], floats[2])
	accel_vec = Vector3(floats[3], floats[4], floats[5])
	accel_angles = accel_vec.angles()

	angles2.roll = (angles2.roll + gyro_delta.roll * dt)

	angles.roll = ((angles.roll + (gyro_delta.roll * dt)) * c) + (accel_angles.roll * (1 - c))

	roll = floats[7]

	print(str(angles.roll) + ' ' + str(gyro_delta.roll) + ' ' + str(accel_angles.roll) + ' ' + str(angles2.roll) + ' ' + str(roll))