#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Antons Rebguns. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
import serial
import time
import struct
from math import sqrt
from binascii import b2a_hex
from chrd_6_const import *

DEG_TO_RAD = 0.017453293 
MILIG_TO_MSS = 0.00980665

MAX_BYTES_SKIPPED = 1000

class CHR6D(object):
    MILIG_TO_MSS = 0.00980665

    MAX_BYTES_SKIPPED = 1000
    def __init__(self,port='/dev/ttyUSB0'):
        self.ser = None
        self.ser = serial.Serial(port)
        self.ser.timeout = 0.015
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.parity = serial.PARITY_NONE
        
        print "Connected to IMU on %s" % port
        
        #self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        #self.yaw_rate = 0.0
        self.pitch_rate = 0.0
        self.poll_rate = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        
        self.imu_data = {
            'timestamp':time.time(),
            'pitch':    0.0,
            'roll':     0.0,
            'gyro_x':    0.0,
            'gyro_y':    0.0,
            'gyro_z':    0.0,
            'accel_x':   0.0,
            'accel_y':   0.0,
            'accel_z':   0.0,
        }
        
    def __del__(self):
        """ Destructor calls self.close_serial_port() """
        self.close()
        
    def calculate_checksum(self, command, data):
        #print "CACLULATE CHECKSUM ________"
        #print "DATA :"
        #print data
        #print "COMMAND :"
        #print command
        chkSum = reduce(int.__add__,map(ord, ['s','n','p']) ,0)
        #print "CHECKSUM :"
        #print chkSum
        #print "str(data) :"
        #print str(data)
        #if isinstance(data, int):
        #    chkSum = chkSum + command + 8 + data #reduce(int.__add__, data, 0)
        #else :
        chkSum = chkSum + command + len(data) + reduce(int.__add__, data, 0)
        return chkSum
        
    def write_to_imu(self, command, data=tuple()):
        
        self.ser.flushInput()
        chkSum = self.calculate_checksum(command, data)
        #print "WRITE TO IMU ________"
        #print "DATA :"
        #print data
        #print "CHECKSUM :"
        #print chkSum
        #print "COMMAND :"
        #print chr(command)
        #if isinstance(data, int):
        #    dataStr = ''.join(chr(data))
        #else:
        dataStr = ''.join(map(chr, data))
        #print "JOINED DATA :"
        #print dataStr
        #print chr(len(dataStr))
        
        #packets : s n p INSTRUCTION LENGTH DATA CHECKSUM
        #bytes:    1 1 1     1           1   N      2
        packetStr = 'snp' + chr(command) + chr(len(dataStr)) + dataStr + chr(chkSum >> 8) + chr(chkSum & 0x0FF)
        #print "PACKET STR :"
        #print packetStr
        self.ser.write(packetStr)
        
         #print 'Command: %s (%s), data packet: %s' % (CODE_TO_STR[command], hex(command).upper(), str(data))
        #wait for response
        time.sleep(0.0054)
        self.read_from_imu()
        
    def read_from_imu(self):
        skipped_bytes = 0
        while skipped_bytes < MAX_BYTES_SKIPPED:
            if self.ser.read() != 's': skipped_bytes += 1
            else : break
        else:
            print 'Unable to findf packet prefix. Throw exception'
            return
        packet = 's' + self.ser.read(2)
        
        if packet !='snp':
            print "Received corrupted packet, prefix was %s" % packet
            return
        
        command = ord(self.ser.read())
        #print "command"
        #print command
        n = ord(self.ser.read())
        #print "n"
        #print n
        dataStr = self.ser.read(n)
        #print "dataStr"
        #print str(dataStr)
        data = map(b2a_hex, dataStr)
        #print "data"
        #print data
        data = map(int, data, [16] * len(data))
        #print data
        #print data[0]
        #print data[1]
        chkSumStr = self.ser.read(2)
        chkSumData = map(b2a_hex, chkSumStr)
        #print "chksumData"
        #print chkSumData
        chkSumData = map(int, chkSumData, [16] * len(chkSumData))
        #print "chkSumData"
        #print chkSumData
        chkSumRx = (chkSumData[0] << 8 |chkSumData[1])
        #print "chkSumRx"
        #print chkSumRx
        chkSum = self.calculate_checksum(command, data)
        #print chkSum
        if chkSumRx != chkSum:
            print "Checksums don't match %d != %d" %(chkSumRx, chkSum)
            return
        
        if command == COMMAND_COMPLETE:
            print 'Command %s (%s) complete' % (CODE_TO_STR[data[0]], hex(data[0]).upper())
        elif command == COMMAND_FAILED:
            print 'Command %s (%s) failed' % (CODE_TO_STR[data[0]], hex(data[0]).upper())
        elif command == BAD_CHECKSUM:
            print 'Bad checksum'
        elif command == BAD_DATA_LENGTH:
            print 'Bad data length for command %s (%s)' % (CODE_TO_STR[data[0]], hex(data[0]).upper())
        elif command == UNRECOGNIZED_PACKET:
            print 'Unrecognized packet %s' % hex(data[0]).upper()
        elif command == BUFFER_OVERFLOW:
            print 'Buffer Overflow'
        elif command == STATUS_REPORT:
            print 'Self test status report'
            if (data[0] >> 5) & 0x01:
                print 'FAILED self-test: gyro_z'
            if (data[0] >> 4) & 0x01:
                print 'FAILED self-test: gyro_y'
            if (data[0] >> 3) & 0x01:
                print 'FAILED self-test: gyro_x'
        elif command == SENSOR_DATA:
            print "SENSOR_DATA"
            self.imu_data['timestamp'] = time.time()
            print data
            #active_channels = (data[0] << 8) | data[1]
            active_channels = data[0]
            print active_channels
            i=1
            #yaw angle
            #if active_channels & 0x8000:
            #    value = sruct.unpack('>h', dataStr[i] + dataStr[i+1])
            #    self.imu_data['yaw'] = value[0] * SCALE_YAW * DEG_TO_RAD
            #    i += 2
            if active_channels & 0x80:
                value = struct.unpack('>h', dataStr[i] + dataStr[i + 1])
                self.imu_data['pitch'] = value[0] * SCALE_PITCH * DEG_TO_RAD
                i += 2
            if active_channels & 0x40:
                value = struct.unpack('>h', dataStr[i] + dataStr[i + 1])
                self.imu_data['roll'] = value[0] * SCALE_ROLL * DEG_TO_RAD
                i += 2
            if active_channels & 0x20:
                value = struct.unpack('>h', dataStr[i] + dataStr[i + 1])
                self.imu_data['gyro_z'] = value[0] * SCALE_GYRO_Z * DEG_TO_RAD
                i += 2
            if active_channels & 0x10:
                value = struct.unpack('>h', dataStr[i] + dataStr[i + 1])
                self.imu_data['gyro_y'] = value[0] * SCALE_GYRO_Y * DEG_TO_RAD
                i += 2
            if active_channels & 0x8:
                value = struct.unpack('>h', dataStr[i] + dataStr[i + 1])
                self.imu_data['gyro_x'] = value[0] * SCALE_GYRO_X * DEG_TO_RAD
                i += 2
            if active_channels & 0x4:
                value = struct.unpack('>h', dataStr[i] + dataStr[i + 1])
                self.imu_data['accel_z'] = value[0] * SCALE_ACCEL_Z * MILIG_TO_MSS
                i += 2
            if active_channels & 0x2:
                value = struct.unpack('>h', dataStr[i] + dataStr[i + 1])
                self.imu_data['accel_y'] = value[0] * SCALE_ACCEL_Y * MILIG_TO_MSS
                i += 2
            if active_channels & 0x1 :
                value = struct.unpack('>h', dataStr[i] + dataStr[i + 1])
                self.imu_data['accel_x'] = value[0] * SCALE_ACCEL_X * MILIG_TO_MSS
                i += 2
        elif command == GYRO_BIAS_REPORT:
            value = struct.unpack('h', dataStr[0] + dataStr[1])
            gyro_z_bias = value[0] * SCALE_GYRO_Z * DEG_TO_RAD
            value = struct.unpack('h', dataStr[2] + dataStr[3])
            gyro_y_bias = value[0] * SCALE_GYRO_Y * DEG_TO_RAD
            value = struct.unpack('h', dataStr[1] + dataStr[5])
            gyro_x_bias = value[0] * SCALE_GYRO_X * DEG_TO_RAD
            print 'gyro biases: (%f, %f, %f)' % (gyro_x_bias, gyro_y_bias, gyro_z_bias)
        elif command == GET_ACCEL_BIAS:
            value = struct.unpack('>h', dataStr[0] + dataStr[1])
            accel_z_bias = value[0] * SCALE_ACCEL_Z * DEG_TO_RAD
            value = struct.unpack('>h', dataStr[0] + dataStr[1])
            accel_y_bias = value[0] * SCALE_ACCEL_Y * DEG_TO_RAD
            value = struct.unpack('>h', dataStr[0] + dataStr[1])
            accel_x_bias = value[0] * SCALE_ACCEL_X * DEG_TO_RAD
            print 'accel biases: (%f, %f, %f)' % (accel_x_bias, accel_y_bias, accel_z_bias)
        elif command == ACTIVE_CHANNEL_REPORT:
            print "Active channel report received"
            #active_channels = data[1] | (data[0] << 8)
            active_channels = data[0]
            print "active channels"
            print active_channels
            if active_channels >> 7 & 0x01:
                print 'pitch channel active'
            if active_channels >> 6 & 0x01:
                print 'roll channel active'
            if active_channels >> 5 & 0x01:
                print 'gyro_z channel active'
            if active_channels >> 4 & 0x01:
                print 'gyro_y channel active'
            if active_channels >> 3 & 0x01:
                print 'gyro_x channel active'
            if active_channels >> 2 & 0x01:
                print 'accel_z channel active'
            if active_channels >> 1 & 0x01:
                print 'accel_y channel active'
            if active_channels >> 0 & 0x01:
                print 'accel_x channel active'
                    
    def close(self):
        if self.ser:
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser.close()
                
    def set_active_channels(self, channels):
        byte2 = str()
        byte2 = '0000000'
        byte = str()
        byte += '1' if channels['pitch'] else '0'
        byte += '1' if channels['roll'] else '0'
        byte += '1' if channels['gyro_z'] else '0'
        byte += '1' if channels['gyro_y'] else '0'
        byte += '1' if channels['gyro_x'] else '0'
        byte += '1' if channels['accel_z'] else '0'
        byte += '1' if channels['accel_y'] else '0'
        byte += '1' if channels['accel_x'] else '0'
        #print "byte :"
        #print byte
        #print byte[3 : ]
        byte = int(byte, 2)
        byte2 = int(byte2, 2)
        #print "BYTE (INT) :"
        #print byte
        #print byte[3]
        self.write_to_imu(SET_ACTIVE_CHANNELS,(byte))
               
    def set_silent_mode(self):
        #Enable silent mode
        self.write_to_imu(SET_SILENT_MODE)
        
    def set_broadcast_mode(self, hz):
        if hz < 20: hz = 20
        if hz > 300: hz = 300
        x = int((hz - 20) / (280.0/255.0))
        self.write_to_imu(SET_BROADCAST_MODE,(x, ))
        
    def set_gyro_bias(self, gyro_x_bias, gyro_y_bias, gyro_z_bias):
        data = struct.pack('>hhh', gyro_z_bias, gyro_y_bias, gyro_x_bias)
        self.write_to_imu(SET_ACCEL_BIAS, map(ord, data))
            
    def zero_rate_gyros(self):
        self.write_to_imu(ZERO_RATE_GYROS)
           
    def self_test(self):
        self.write_to_imu(SELF_TEST)
            
    def set_gyro_scale(self, gyro_x_scale, gyro_y_scale, gyro_z_scale):
        data = struct.pack('>fff', gyro_z_scale, gyro_y_scale, gyro_x_scale)
        self.write_to_imu(SET_GYRO_SCALE, map(ord, data))
            
    def write_to_flash(self):
        self.write_to_imu(WRITE_TO_FLASH)
            
    def get_data(self):
        self.write_to_imu(GET_DATA)
        return self.imu_data
    
    def get_active_channels(self):
        self.write_to_imu(GET_ACTIVE_CHANNELS)
        
    def get_broadcast_mode(self):
        self.write_to_imu(GET_BROADCAST_MODE)
    
    def get_accel_bias(self):
        self.write_to_imu(GET_ACCEL_BIAS)

    def get_gyro_bias(self):
        self.write_to_imu(GET_GYRO_BIAS)
        
    def get_gyro_scale(self):
        self.write_to_imu(GET_GYRO_SCALE)
        
    def read_accel_angrate_orientation(self):
        while self.ser.read() != 's':
            print "looking for packet prefix 'snp'"
        packet = 's' + self.ser.read(2)
        
        if packet != 'snp':
            print "Received corrupted packet, prefix was %s" % packet
            return
        
        command = ord(self.ser.read())
        #print "packet type = %s" % hex(command).upper()
        n = ord(self.ser.read())
        #print "data bytes = %d" % n
        
        dataStr = self.ser.read(n)
        data = map(b2a_hex, dataStr)
        data = map(int, data, [16]*len(data))
        #print data
        
        chkSumStr = self.ser.read(2)
        chkSumData = map(b2a_hex, chkSumStr)
        chkSumData = map(int, chkSumData, [16] * len(chkSumData))
        chkSumRx = (chkSumData[0] << 8) | chkSumData[1]
        chkSum = self.calculate_checksum(command, data)
        
        if chkSumRx != chkSum:
            print "Checksums don't match %d != %d" % (chkSumRx, chkSum)
            return
        
        res = {}
        
        if command == SENSOR_DATA:
            res['timestamp'] = time.time()
            active_channels = (data[0] <<8 | data[1])
            
            i = 2
            
            if active_cahnnels & 0x80:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                pitch = value[0] * SCALE_PITCH * DEG_TO_RAD
                res['pitch'] = pitch
                i+=2
                
            if active_cahnnels & 0x40:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                roll = value[0] * SCALE_ROLL * DEG_TO_RAD
                res['roll'] = roll
                i+=2
            
            if active_cahnnels & 0x20:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                gyro_z = value[0] * SCALE_GYRO_Z * MILIG_TO_MSS
                res['pitch'] = pitch
                i+=2
            
            if active_cahnnels & 0x80:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                gyro_y = value[0] * SCALE_GYRO_Z * DEG_TO_RAD
                res['pitch'] = pitch
                i+=2
        
            if active_cahnnels & 0x80:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                gyro_x = value[0] * SCALE_GYRO_X * DEG_TO_RAD
                res['pitch'] = pitch
                i+=2
                
            if active_cahnnels & 0x20:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                accel_z = value[0] * SCALE_ACCEL_Z * MILIG_TO_MSS
                res['pitch'] = pitch
                i+=2
            
            if active_cahnnels & 0x80:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                accel_y = value[0] * SCALE_ACCEL_Z * DEG_TO_RAD
                res['pitch'] = pitch
                i+=2
        
            if active_cahnnels & 0x80:
                value = struct.unpack('>h', dataStr[i] + dataStr[i+1])
                accel_x = value[0] * SCALE_ACCEL_X * DEG_TO_RAD
                res['pitch'] = pitch
                i+=2
        return res
        
if __name__ == "__main__":
    sio = CHR6D()
    sio.set_silent_mode
    ch = {  'pitch':1,
            'roll': 1,
            'gyro_x':1,
            'gyro_y':1,
            'gyro_z':1,
            'accel_x':1,
            'accel_y':1,
            'accel_z':1,
        }
    sio.set_active_channels(ch)
    
    sio.get_active_channels()
    sio.get_broadcast_mode()
    sio.self_test()
    sio.get_data()
