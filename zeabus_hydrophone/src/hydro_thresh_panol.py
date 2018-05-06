#!/usr/bin/python

import struct
import numpy
import math
import time
import serial
import numpy as np

ser = serial.Serial('/dev/usb2serial/ftdi_A1041SI3', 115200)
def float_hex4(f):
    return ''.join(('%2.2x'%ord(c)) for c in struct.pack('<f', f))
def uint_hex4(I):
    return ''.join(('%2.2x'%ord(c)) for c in struct.pack('<I', I))
def set(mode,value):
    res = []
    res.append('\xff')
    res.append('\xff')
    #res.append(mode)
    if mode==0x00 or mode==0x01 or mode==0x04:
        print "uint32"
        if mode==0x00:
            res.append('\x00')
        elif mode==0x01:
            res.append('\x01')
        elif mode==0x04:
            res.append('\x04')
        tmp = uint_hex4(value)
        res.append(tmp[0:2].decode("hex"))
        res.append(tmp[2:4].decode("hex"))
        res.append(tmp[4:6].decode("hex"))
        res.append(tmp[6:8].decode("hex"))
    elif mode==0x02 or mode==0x03 or mode == 0x09:
        print "float"
        if mode==0x02:
            res.append('\x02')
        elif mode==0x03:
            res.append('\x03')
        elif mode == 0x09:
            res.append('\x09')
        tmp = float_hex4(value)
        res.append(tmp[0:2].decode("hex"))
        res.append(tmp[2:4].decode("hex"))
        res.append(tmp[4:6].decode("hex"))
        res.append(tmp[6:8].decode("hex"))
    res.append('\x00')
    #print list(array.array('B', res).tostring())
    print res
    return ''.join(res)

if __name__ == '__main__':
    print('Send adjust front threshold')
    res = set(0x02, 0.5)
    ser.write(res)
    ser.flush()
    ret = ser.read(46)
    #ret = '\xff\xff\x1e\x00\xcd\xcc\xcc\x3d\xcd\xcc\x4c\x3e\xb8\x88\x00\x00\xc8\x00\x00\x00\x01\x00\x00\x00\xcd\xcc\x4c\x3e\xcd\xcc\x4c\x3e\xdf\x07\x00\x00\xcd\xcc\x4c\x3e\xff\xf0'
         
    print 'return ack = %s, ack len = %d' %(ret, len(ret))
    len = struct.unpack("<H", ''.join(ret[2:4]))[0]
    print 'Length = %d' %(len)
 
    front = struct.unpack("<f", ''.join(ret[4:8]))[0]
    print 'front = %f' %(front)

    pwr = struct.unpack("<f", ''.join(ret[8:12]))[0]
    print 'pwr thres = %f' %(pwr)

    freq = struct.unpack("<I", ''.join(ret[12:16]))[0]
    print 'freq = %d' %(freq)

    delay = struct.unpack("<I", ''.join(ret[16:20]))[0]
    print 'delay = %d' %(delay)

    speed = struct.unpack("<I", ''.join(ret[20:24]))[0]
    print 'speed = %d' %(speed)

    auto = struct.unpack("<I", ''.join(ret[24:28]))[0]
    print 'auto = %d' %(auto)
   
    gain_L = struct.unpack("<f", ''.join(ret[28:32]))[0]
    print 'gain_L = %f' % (gain_L)
    
    gain_R = struct.unpack("<f", ''.join(ret[32:36]))[0]
    print 'gain_R = %f' % (gain_R)

    algorithm = struct.unpack("<I", ''.join(ret[36:40]))[0]
    print 'algorithm = %d' % (algorithm)
    
    cfar = struct.unpack("<f", ''.join(ret[40:44]))[0]
    print 'cfar = %f' % (cfar)

