#!/usr/bin/env python

import struct
import numpy
import math
import time
import serial
import numpy as np
import scipy.stats as stats
from zeabus_hydrophone.srv import hydro_info
from zeabus_hydrophone.msg import hydro_msg
from std_msgs.msg import Bool
import rospy
ser = serial.Serial('/dev/ttyUSB0', 115200 )
#ser = serial.Serial('/dev/usb2serial/ftdi_AJ038YFU', 115200)
# ser.readline()
############################INITIAL GLOBALVARIABLE#################################################
fre, set_fre = 0, 40000 #SET FREQUENCY
k = 0
status = False
sending = False
check_elv = True
num_elv = 0
x_min,x_max = 0,0
##############################################################################



######################################SENDING#######################################
def float_hex4(f):
    return ''.join(('%2.2x'%ord(c)) for c in struct.pack('<f', f))
def uint_hex4(I):
    return ''.join(('%2.2x'%ord(c)) for c in struct.pack('<I', I))
# def scopemode():
#     return []
def set(mode,value): #send to board 
    global ser
    res = []    #EMPTY LIST
    res.append('\xff')
    res.append('\xff')
    #res.append(mode)
    if mode==0x00 or mode==0x01 or mode==0x04 or mode==0x05 or mode==0x06 or mode==0x07 or mode==0x08 :
        print "uint32" #Uint type
        if mode==0x00:
            res.append('\x00')
        elif mode==0x01:
            res.append('\x01')
        elif mode==0x04:
            res.append('\x04')
        elif mode==0x05:
            res.append('\x05')
        elif mode==0x06:
            res.append('\x06')
        elif mode==0x07:
            res.append('\x07')
        elif mode==0x08:
            res.append('\x08')        
        tmp = uint_hex4(value) 
        res.append(tmp[0:2].decode("hex"))
        res.append(tmp[2:4].decode("hex"))
        res.append(tmp[4:6].decode("hex"))
        res.append(tmp[6:8].decode("hex"))
    elif mode==0x02 or mode==0x03 or mode==0x09 :
        print "float"
        if mode==0x02:
            res.append('\x02')
        elif mode==0x03:
            res.append('\x03')
        elif mode==0x09:
            res.append('\x09')
        tmp = float_hex4(value)
        res.append(tmp[0:2].decode("hex"))
        res.append(tmp[2:4].decode("hex"))
        res.append(tmp[4:6].decode("hex"))
        res.append(tmp[6:8].decode("hex"))
        
    # elif mode == 0x08 or mode == 0x09:
    #     print "scope mode"
    #     if mode==0x08:
    #         res.append('\x08')
    #     elif mode==0x09:
    #         res.append('\x09')
    #     tmp = float_hex4(value)
    #     res.append(tmp[0:2].decode("hex"))
    #     res.append(tmp[2:4].decode("hex"))
    #     res.append(tmp[4:6].decode("hex"))
    #     res.append(tmp[6:8].decode("hex"))
        
        #tmp = scopemode()
    res.append('\x00')    
    #print list(array.array('B', res).tostring())
    print res
    #return ''.join(res)
    ser.write(res)
    ser.flush()
    return res
###################################PARTICLE FILTER################################################################


class particleFilter:
    def __init__(self,N,x_min,x_max,f,h,simga_r):
        # print "__________________________________PARTICLE__________________________________________"
        nx = x_min.shape[0]     #number of element 
        d = np.random.rand(nx,N) #random 3 to N
        rang_x = (x_max-x_min).reshape((nx,1)) 
        rang_x = np.repeat(rang_x,N,1)
        x_min = x_min.reshape((nx,1))
        x_min = np.repeat(x_min,N,1)
        state_x = d*rang_x+x_min
        self.N = N
        self.state_x = state_x
        # print "NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN"
        # print self.N
        self.weight = (1./float(N))*np.ones((N,))
        # print "self weight"
        # print self.weight
        self.predict_function = f
        self.observation_function = h
        self.sigma_r = simga_r
       # sef.sigma_r[0] = 2.0
       # sef.sigma_r[4] = 2.0;
        self.nx = nx
    def reset(self,x_min,x_max) :
        nx = x_min.shape[0]  
        N = self.N
        d = np.random.rand(nx,N) #random 3 to N
        rang_x = (x_max-x_min).reshape((nx,1)) 
        rang_x = np.repeat(rang_x,N,1)
        x_min = x_min.reshape((nx,1))
        x_min = np.repeat(x_min,N,1)
        state_x = d*rang_x+x_min
        self.state_x = state_x
        self.weight = (1./float(N))*np.ones((N,))
    def predict(self,v):
        state_x_km1 = self.state_x
        state_x_k = self.predict_function(state_x_km1,v)
        self.state_x = state_x_k
    def update_weight(self,z_obv):
        # print "+++++++++++++++++++++++++++++++++++++++UPDATE WEIGHT+++++++++++++++++++++++++++++++++++++"
        nx = self.nx
        ny = z_obv.shape[0]
        N = self.N
        state_x = self.state_x
        mu = self.observation_function(state_x)
        z= z_obv.reshape((ny,1))
        # print "==z=="
        # print z
        z = np.repeat(z,N,1)
        z = z -mu
        # print "z"
        # print z
        z2 = np.linalg.solve(self.sigma_r,z)
        # print "z2"
        # print z2
        e = 0.5*(z*z2).sum(0) + 0.5*np.log(np.linalg.det(self.sigma_r))
        # print "e"
        # print e        
        prob = np.exp(-e)
        # print "prob"
        # print prob
        w = self.weight
        # print "1"
        # print w 
        w = w*prob
        # print "2"
        # print w 
        w = w/w.sum()
        self.weight = w
        # print "this is W"
        # print w
    def resampling(self):
        # print "GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG"
        N = self.N
        w = self.weight
        nx = self.nx
        Neff = 1./(w**2).sum()
        # print "YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY"
        # print w   ######################### 
        if Neff < (2./3.)*N : # resampling
            w_cum = w.cumsum()
            d = np.random.rand(N,)
            x_new = np.zeros((nx,N))
            x_old = self.state_x
            self.weight = (1./float(N))*np.ones((N,))
            for k in range(N):
                dk = d[k]
                b = np.nonzero(w_cum>dk)
                b = b[0]
                b = b.min()
                x_new[:,k] = x_old[:,b]
            self.state_x = x_new
            # print "==x_new=="
            # print x_new
    def get_mmse(self):
        x = self.state_x
        w = self.weight
        xmmse = (w*x).sum(1)
        return xmmse

def fx(x):
    x[-1] = 0
    return x

def fx2(x,y):
    pi = np.pi
    z = x+y
    z[0] = z[0]#%(2*np.pi)
    z[0] = z[0]*(z[0]<= pi) +(z[0]-2*pi)*(z[0]>pi)
    z[0] = z[0]*(z[0]> -pi) +(z[0]+2*pi)*(z[0]<=-pi)
    z[1] = z[1]*(z[1]<= pi/2.0) +(pi-z[1])*(z[1]>pi/2.0)
    z[1] = z[1]*(z[1]>=0) +(-z[1])*(z[1]<0)
    z[2] = z[2]#%(2*np.pi)
    z[2] = z[2]*(z[2]<= pi) +(z[2]-2*pi)*(z[2]>pi)
    z[2] = z[2]*(z[2]> -pi) +(z[2]+2*pi)*(z[2]<=-pi)
    return z

def hx(x):
    global fre, set_fre
    az = x[0]
    el = x[1]
    c  = x[2]
    if x.ndim > 1:
        nx,N = x.shape
    else:
        N = 1
    if fre == 0:      #check error
        print "recheck"
        fre = set_fre  
    fs = fre/1000.0
    lamb = 1500.0/fs   #26C
    phase = np.array([np.cos(az)*np.sin(el),np.sin(az)*np.sin(el),c])
    ant = np.array([[10,10],[-10,10],[-10,-10],[10,-10]])     #np.array([[10,10],[-10,10],[-10,-10.],[10,-10]])
    ant_phase = ant*np.pi*2/lamb
    A = np.ones((4,3))
    A[:,:2] = ant_phase
    angles = np.dot(A,phase)
    cr = np.cos(angles)
    ci = np.sin(angles)
    if x.ndim > 1:
        c = np.zeros((8,N))
        c[:4,:] = cr
        c[4:,:] = ci
    else:
        c = np.zeros((8,))
        c[:4] = cr
        c[4:] = ci
    return c
###################################READING#########################################
class readdata:
    def __init__(self):
        self.az_t = 0
        self.elv_t = 0
        self.pre_azi = 0
        self.pre_elv = 0
    def getData(self):
        global fre, pf, k, sending, check_elv, check_stop, num_elv
        global x_max,x_min
        count, x,c = 0, 0 ,0 
        sending = False        
        data = hydro_msg()

        # print "Test"

        if check_elv :    #Check elv_t < 25 three time? [N] --> check_elv = True
            data.stop = False
        i = 1
        while True: 
            # print '0'
            #print ser
            x = ser.read(1) 
            # print '1'
            # print x
            if x=='\xff':
                x = ser.read(1)
                # print '2'
                count +=1
               # print repr(x)

                if x=='\xff': 
                    x = ser.read(85)
                    #print x
                    #print "3"
                    count = 0
                    break        
            if count  == 2:# wrong header
                # print "3"
                data.azi = self.pre_azi   ######self.az_t 
                data.elv = self.pre_elv   ######self.elv_t
                data.distance = -999    ####data.stop = True#######
                count = 0
                # print fre
                # print "===== Pulse Error ======"    
          
        # len = struct.unpack("<H", ''.join(x[0:2]))[0]    
        seq = struct.unpack("<H", ''.join(x[2:4]))[0]    #H means short int type hexa decimal
        azi = struct.unpack("<f", ''.join(x[4:8]))[0]
        elv = struct.unpack("<f", ''.join(x[8:12]))[0]
        itv = struct.unpack("<H", ''.join(x[12:14]))[0]
        pct = struct.unpack("<H", ''.join(x[14:16]))[0]
        po1 = struct.unpack("<f", ''.join(x[16:20]))[0]
        po2 = struct.unpack("<f", ''.join(x[20:24]))[0]
        po3 = struct.unpack("<f", ''.join(x[24:28]))[0]
        po4 = struct.unpack("<f", ''.join(x[28:32]))[0]
        stt = struct.unpack("<c", ''.join(x[32:33]))[0]
        fre = struct.unpack("<I", ''.join(x[33:37]))[0]
        ph1 = struct.unpack("<f", ''.join(x[37:41]))[0]
        ph2 = struct.unpack("<f", ''.join(x[41:45]))[0]
        ph3 = struct.unpack("<f", ''.join(x[45:49]))[0]
        ph4 = struct.unpack("<f", ''.join(x[49:53]))[0]
        c1r = struct.unpack("<f", ''.join(x[53:57]))[0]
        c1i = struct.unpack("<f", ''.join(x[57:61]))[0]
        c2r = struct.unpack("<f", ''.join(x[61:65]))[0]
        c2i = struct.unpack("<f", ''.join(x[65:69]))[0]
        c3r = struct.unpack("<f", ''.join(x[69:73]))[0]
        c3i = struct.unpack("<f", ''.join(x[73:77]))[0]
        c4r = struct.unpack("<f", ''.join(x[77:81]))[0]
        c4i = struct.unpack("<f", ''.join(x[81:85]))[0]
        #rsrv = struct.unpack("<c", ''.join(x[85:89]))[0]
       
        # print "freq = %d" %(fre)
        # print "seq : %d" %seq
        
        # if c1r or c1i or c2r or c2i or c3r or c3i or c4r or c4i == 0:
        #     print "some of c is zero.!!!!!"
        #     sending = False 
        
        ##########################################################################################
        if fre == set_fre: #tell me this is frequency that i want 
            sending = True
        ###########################################################################################
        # if azi == 0 or elv == 0:
        #     print "Error : Azimuth or Elvation from DSP Board"
        #     sending = False
        #     return
        
        # if np.abs(c1r) > 1 or np.abs(c1i) > 1 or np.abs(c2r) > 1 or np.abs(c2i) > 1 or np.abs(c3r) > 1 or np.abs(c3i) > 1 or np.abs(c4r) > 1 or np.abs(c4i) > 1:
        #     print "Error : cout more than 1"
        #     sending = False
        #     return
       
        m1 = c1r+1j*c1i + 1e-6 #le - 6 
        m2 = c2r+1j*c2i + 1e-6
        m3 = c3r+1j*c3i + 1e-6
        m4 = c4r+1j*c4i + 1e-6
        # print "c1 = %.6f, c2 = %.6f, c3 = %.6f, c4 = %.6f" % (c1,c2,c3,c4)
        cp = np.array([(c1r)/np.abs(m1), (c2r)/np.abs(m2), (c3r)/np.abs(m3), (c4r)/np.abs(m4), (c1i)/np.abs(m1), (c2i)/np.abs(m2), (c3i)/np.abs(m3), (c4i)/np.abs(m4)])

        # print "po1 = %e || po2 = %e || po3 = %e || po4 = %e" %(po1,po2,po3,po4)
        # print "c1r = %e || c1i = %e ||c2r = %e || c2i = %e ||c3r = %e || c3i = %e ||c4r = %e || c4i = %e" %(c1r,c1i,c2r,c2i,c3r,c3i,c4r,c4i)
        # print ""
        # print "m1=%f m2=%f m3=%f m4=%f" %(np.abs(m1),np.abs(m2),np.abs(m3),np.abs(m4))
        # #################################################################################################
        # print cp ########################################################################################
        if fre == set_fre  :
            # print "particleFilter"
            v = np.random.rand(3,N)*2.0-1.0
            v[0] = v[0]*pi/9.0 #16.0   #trace max speed 
            v[1] = v[1]*pi/9.4 #23-1507 
            v[2] = v[2]*pi/3.0
            pf.predict(v)
            pf.update_weight(cp)
            pf.resampling()
            xk = pf.get_mmse()
            self.az_t = xk[0]*180./pi
            self.elv_t = xk[1]*180./pi
            c = xk[2]
            # print "===xk==="
            # print xk
            print "freq = %d" %(fre)
            print "seq : %d" %seq
            print "c1r = %e || c1i = %e" %(c1r,c1i)
            print "c2r = %e || c2i = %e" %(c2r,c2i)
            print "c3r = %e || c3i = %e" %(c3r,c3i)
            print "c4r = %e || c4i = %e" %(c4r,c4i)

            print "angle c1 = %f" %(np.angle(m1, True))
            print "angle c2 = %f" %(np.angle(m2, True))
            print "angle c3 = %f" %(np.angle(m3, True))
            print "angle c4 = %f" %(np.angle(m4, True))



        # checknan = is_nan(float(azi)) #can't pass
        # if checknan : 
        #     print "___________________________________________________________________________________________________________"
        #     print "%f"%self.az_t
        #     print "___________________________________________________________________________________________________________"
        #     print status
        #     pf.reset(x_min,x_max)
        #     check_elv = True
        #     status = False
        #     check_stop = False
        #     num_elv = 0   ###!!!!!!!!!!!
        #     print "--- RESET ---"
        #     d = rd.getData()
        # print "c1r = %e || c1i = %e ||c2r = %e || c2i = %e ||c3r = %e || c3i = %e ||c4r = %e || c4i = %e" %(c1r,c1i,c2r,c2i,c3r,c3i,c4r,c4i)
        # print "Particle Filter Az: %.2f,Elv: %.2f,Ot: %.3f ,Freq : %.0f\n" % (self.az_t, self.elv_t,c,fre/1000)
        # print "Azi : %f, Elv : %f" %(azi,elv)

        if self.elv_t < 25:
            if num_elv < 2:
                num_elv += 1
            else:
                check_elv = False
                data.stop = True
                check_stop = True
        else:
            num_elv = 0

        self.pre_azi = self.az_t
        self.pre_elv = self.elv_t

        data.azi = self.az_t
        data.elv = self.elv_t
        data.freq = fre       
        return data

# def is_nan(x): #check nan-value ..... i think it isn't work 
#     return (x is np.nan or x != x)

def callback(msg):
    global status
    status = msg.data

###############################MAIN############################################################################
if __name__ == '__main__':
    global pf, status, sending, check_elv, check_stop, set_fre
    R = np.eye(8)*0.1
    R[0,0] = 2.0
    R[4,4] = 2.0
    pi = np.pi
    x_min = np.array([-pi,0,-pi])
    x_max = np.array([pi,pi/2.,pi])
    N= 1100
    pf = particleFilter(N,x_min,x_max,fx2,hx,R)
    rd = readdata()

    rospy.init_node('HYDROPHONE_2016')
    pub = rospy.Publisher('hydro', hydro_msg, queue_size =  4)
    rospy.Subscriber('/hydro_status',Bool,callback)

    print "READY TO GET THE DATA"
   
    res = set(0x00,2000)   #Delay observer (2000 mS)
    res = set(0x01,set_fre)  #Frequency
    res = set(0x02,0.6)   #front threshole  
    res = set(0x03,0.02)   #Power Threshold (dB)   
    res = set(0x04,1496)   #Acoustic sound speed (1496 m/S)
    # res = set(0x08,0.0) #process mode #not avalible 
    # res = set(0x09,0.0) #scope mode  #not avalible
    res = set(0x08,2016) # 2015 = Algor 2015(detect)|| 2016= Alogor 2016 (Cfar detection) || 1= Algor 2016(pulse detection)
    res = set(0x09,100.0) # Cfar power at least 20.0    
    """##ADD##
    res = set(0x05,0)      #Auto Frequence Enable / Disable (0 : disable, other : enable)
    res = set(0x06,0)      #Gain L (0-255) Max  1V
    res = set(0x07,0)      #Gain R (0-255) Max  1V
    """
       
    while not rospy.is_shutdown() :
        if status == True:
            print status
            pf.reset(x_min,x_max)
            check_elv = True
            status = False
            check_stop = False
            num_elv = 0   ###!!!!!!!!!!!
            print "--- RESET ---"
        re = rd.getData()
        if(sending):
            print "==== Sending ===="
            pub.publish(re)
        # else:
            # print "==== Fail ====="