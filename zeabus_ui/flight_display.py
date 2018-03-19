#!/usr/bin/env python
from std_msgs.msg import Float64
import rospy
from sensor_msgs.msg import Imu,Joy
import math
import pygtk
import gtk, gobject, cairo
from gtk import gdk
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import cv2
depth = 0
offset = 0
roll = 0.0
posex = 0
posey = 0
pitch =float(0)
yaw = float(0)
omg =float(0)
ini = [False,245,-230]
#rospy.init_node("node")
class Screen( gtk.DrawingArea ):
    """ This class is a Drawing Area"""
    def __init__(self):
        super(Screen,self).__init__()
        self.connect ( "expose_event", self.do_expose_event )
        self.connect ( "motion_notify_event", self._mouseMoved )
        self.connect ( "button_press_event", self.key_press_event )
        self.add_events (  gdk.BUTTON_PRESS_MASK |   gdk.BUTTON_RELEASE_MASK |   gdk.POINTER_MOTION_MASK )

        gobject.timeout_add( 50, self.tick )

    def tick ( self ):
        ## This invalidates the screen, causing the expose event to fire.
        self.alloc = self.get_allocation ( )
        rect = gtk.gdk.Rectangle ( self.alloc.x, self.alloc.y, self.alloc.width, self.alloc.height )
        self.window.invalidate_rect ( rect, True )
        return True # Causes timeout to tick again.

    ## When expose event fires, this is run
    def _mouseMoved(self,widget,event):
        self.x = event.x-350
        self.y = event.y-300
    def key_press_event(self,widget,event):
        global offset,yaw,ini
        if 220<self.y<270:
            offset = yaw
        if 200<self.x<300 and -280<self.y<-180:
            ini = [False,self.x,self.y]
        if -30<self.x<30 and -30<self.y<30:#re ini
            ini[0] = True
    def do_expose_event( self, widget, event ):

        self.cr = self.window.cairo_create( )
        self.cr2 = self.window.cairo_create( )
        self.cr3 = self.window.cairo_create( )
        

        self.draw( *self.window.get_size( ) )
        self.draw3( *self.window.get_size( ) )
        self.draw2( *self.window.get_size( ) )

        self.draw4( *self.window.get_size( ) )

class MyStuff ( Screen ):
    """This class is also a Drawing Area, coming from Screen."""
    def __init__ ( self ):
        Screen.__init__( self )
        self.x, self.y = 0, -0
        self.rot = 0
        self.rot2 = 0
        self.sx, self.sy = 1, 1

    def draw4(self,w,h):

        global x,y,depth , roll ,pitch,yaw
        cr = self.cr2

        cr.set_source_rgb(1, 1, 1)
        cr.select_font_face("Courier", cairo.FONT_SLANT_NORMAL,
        cairo.FONT_WEIGHT_BOLD)
        cr.set_font_size(18)

        data = [
                [ "x",x],
                [ "y",y],
                [ "z",depth],
                [ "r",roll*180.0/math.pi],
                [ "p",pitch*180.0/math.pi],
                [ "y",yaw]
               ]

        start_x,start_y = w/2.0 - 120, -60
        offset_y = 0
        for label , value in data:
            cr.move_to(start_x,start_y+offset_y)
            cr.show_text("%s:%7.2f"%(label,value))

            if label== data[2][0]: offset_y += 20


            offset_y += 20



    def draw( self, width, height ):
        global roll,pitch,yaw
        cr = self.cr
        matrix = cairo.Matrix ( 1, 0, 0, 1, width/2, height/2 )
        cr.transform ( matrix ) # Make it so...
        cr.save ( )

        ThingMatrix = cairo.Matrix ( 1, 0, 0, 1, 0, 0 )
        cairo.Matrix.translate(ThingMatrix,0,-pitch*180*(100.0/15)/math.pi)#yaw pitch
        cairo.Matrix.rotate( ThingMatrix, self.rot ) # Do the rotation
        cr.transform ( ThingMatrix ) # and commit it to the context
        self.drawScale(cr)
        self.drawCairoStuff ( self.cr,width,height )
        self.rot = -roll
        self.drawScale(cr)
        self.drawyawn(cr)
        cr.restore ( )
        self.drawface(cr,width,height)

    def draw2(self, width, height):
        global roll,pitch,yaw,offset

        cr2 = self.cr2
        matrix = cairo.Matrix(1,0,0,1,width/2, height/2)
        cr2.transform(matrix)
        cr2.save()
        ThingMatrix = cairo.Matrix(1,0,0,1,0,0)
        cairo.Matrix.translate(ThingMatrix,-yaw*10-(offset*10),0)#yaw pitch
        cr2.transform ( ThingMatrix ) # and commit it to the context
        self.drawyawn2(cr2,width,height)
        cr2.restore()
        self.drawmask(cr2,width,height)
        self.drawline(cr2)

    def draw3(self,width,height):#depth
        global roll,pitch,yaw,depth
        cr3 = self.cr3
        matrix = cairo.Matrix(1,0,0,1,width/2,height/2)
        cr3.transform(matrix)
        cr3.save()
        ThingMatrix = cairo.Matrix(1,0,0,1,0,0)
        cairo.Matrix.translate(ThingMatrix,0,depth*100)#' ' depth
        cr3.transform(ThingMatrix)
        self.drawdepth(cr3,width,height)
        cr3.restore()


    def drawline(self,cr):
        global ini,depth,posex,posey
        cr.set_source_rgb(1,1,1)
        cr.rectangle(350-150,-300+20,100,100)
        cr.fill()
        cr.set_source_rgb(1,0,0)
        if ini[0] == False:
            self.x = ini[1]
            self.y = ini[2]
        else:
            posex=0
            posey=0
        cr.rectangle(self.x+posex,self.y+posey,8,8)
        cr.fill()
        cr.set_source_rgb(0,0,0)
        cr.rectangle(self.x,self.y,8,8)
        cr.fill()

    def drawCairoStuff ( self, cr ,w,h):
        #background
        cr.rectangle(-w*5,-h*5,w*20,h*20)
        cr.set_source_rgb( 0, 0.3, 0.7)
        cr.fill( )
        #foreground
        cr.rectangle( -w*5, 0, w*40, w*40 )
        cr.set_source_rgb( 0.4, 0.3, 0.1)
        cr.fill( )
        cr.set_source_rgb( 1, 1, 1 )
        cr.stroke( )
    def drawScale(self,cr):
        #gaugeScale
        m=0
        for x in range(10,2000,10):
            m += 5
            w = 10+ ( m % 2 * 10 )
            if x%100 == 0: w = 50
            cr.rectangle ( -w/2, x, w, 1 )
            cr.rectangle ( -w/2, -x , w, 1 )
        cr.set_source_rgb ( 0,0,0 )
        cr.fill()
        #label
        cr.set_source_rgb(0.1, 0.1, 0.1)

        cr.select_font_face("Courier", cairo.FONT_SLANT_NORMAL,
        cairo.FONT_WEIGHT_BOLD)
        cr.set_font_size(20)
        tmp = 15
        for i in range(1,600): ##scale text
            cr.move_to(27,i*100)
            cr.show_text('-'+str(tmp))
            cr.move_to(27,-i*100)
            cr.show_text(str(tmp))
            tmp+=15

    def drawyawn(self,cr):

        for x in range(10,2000,2):
            w = 0
            if x%100 == 0: w = 50
            cr.rectangle ( x,0,2,w/2 )
            cr.rectangle ( -x, 0, 2, w/2 )
        cr.set_source_rgb ( 195,44,44 )
        cr.fill()
        cr.set_source_rgb(195,44,44)
        cr.move_to(-10000,0)
        cr.line_to(10000,0)
        cr.stroke()

    def drawyawn2(self,cr,width,height):
        cr.rectangle(-width*20,220,width*40,50)
        cr.set_source_rgb(0,0.1,0)
        cr.fill()
        cr.set_source_rgb(0.7,0.9,0)
        cr.move_to(-10000,220)
        cr.line_to(10000,220)
        cr.move_to(-10000,270)
        cr.line_to(10000,270)
        cr.stroke()
        for x in range(0,3601,2):
            w = 0
            if x%100 == 0: w = 50
            elif x%50 == 0: w = 25
            cr.rectangle ( x,220,2,w/2 )
            cr.rectangle ( -x, 220, 2, w/2 )
        cr.select_font_face("Courier", cairo.FONT_SLANT_NORMAL,cairo.FONT_WEIGHT_BOLD)
        cr.set_font_size(20)
        tmp = 10
        for i in range(1,37):
            cr.set_source_rgb(0.7,0.9,0)
            cr.move_to(-i*100,260)
            cr.show_text('-'+str(tmp))
            cr.move_to(i*100,260)
            cr.show_text(str(tmp))
            tmp += 10
        cr.set_source_rgb ( 0.7,0.9,0 )
        cr.fill()

    def drawdepth(self,cr3,width,height):
        cr3.rectangle(-350,-100000,70,200000)
        cr3.set_source_rgb(0,0.1,0)
        cr3.fill()
        cr3.rectangle(-350,-150,70,300)
        cr3.set_source_rgb(0.9,0.1,0)
        cr3.fill()
        for x in range(10,20000,10):
            w = 0
            if x%100 == 0: w = 25
            elif x%50 == 0: w = 15
            cr3.rectangle ( -350, -x , w, 2 )
            cr3.rectangle ( -350, x , w, 2 )
        cr3.set_source_rgb ( 0.7,0.9,0 )
        cr3.fill()
        cr3.select_font_face("Courier", cairo.FONT_SLANT_NORMAL,cairo.FONT_WEIGHT_BOLD)
        cr3.set_font_size(17)
        tmp = 1
        for i in range(1,600):
            cr3.set_source_rgb(0.7,0.9,0)
            cr3.move_to(-330,-i*100)
            cr3.show_text(' '+str(tmp)+'m')
            cr3.move_to(-330,i*100)
            cr3.show_text('-'+str(tmp)+'m')
            tmp += 1


    def drawmask ( self, ctx, width,height ):
        ctx.set_source_rgb ( 0, 0, 0 )
        ctx.arc ( 0, -40,160*math.sqrt(2),math.pi,0 )
        ctx.line_to(width,-40)
        ctx.line_to(width,-height)
        ctx.line_to(-280,-height)
        ctx.line_to(-280,-40)
        ctx.set_source_rgb ( 0, 0, 0)
        ctx.fill()
        ctx.set_source_rgb ( 1, 1, 1 )
        ctx.arc ( 0, -40,160*math.sqrt(2),math.pi,0 )
        ctx.stroke()

        ctx.arc ( 0, -10,160*math.sqrt(2),0,math.pi )
        ctx.line_to(-280,-10)
        ctx.line_to(-280,220)
        ctx.line_to(width,220)
        ctx.line_to(width,-10)
        ctx.set_source_rgb ( 0, 0, 0)
        ctx.fill()
        ctx.set_source_rgb ( 1, 1, 1 )
        ctx.arc ( 0, -10,160*math.sqrt(2),0,math.pi )
        ctx.stroke()

        ctx.move_to(-225,-100)
        ctx.line_to(-225,270)
        ctx.line_to(225,270)
        ctx.line_to(225,-100)
        ctx.line_to(width,-100)
        ctx.line_to(width,height)
        ctx.line_to(-280,height)
        ctx.line_to(-280,-60)
        ctx.set_source_rgb(0,0,0)
        ctx.fill()
        #mask depth
        ctx.move_to(-280,200)
        ctx.line_to(-width,200)
        ctx.line_to(-width,-height)
        ctx.line_to(-280,height)
        ctx.close_path()
        ctx.set_source_rgb(0,0,0)
        ctx.fill()
        #mask depth
        ctx.move_to(-280,-240)
        ctx.line_to(-width,-240)
        ctx.line_to(-width,-height)
        ctx.line_to(-280,-height)
        ctx.close_path()
        ctx.set_source_rgb(0,0,0)
        ctx.fill()
        #line
        ctx.set_source_rgb(1,1,1)
        ctx.move_to(-225,-70)
        ctx.line_to(-225,19)
        ctx.move_to(225,-70)
        ctx.line_to(225,19)
        ctx.stroke()
        ctx.set_source_rgb(0.7,0.9,0)
        ctx.move_to(225,220)
        ctx.line_to(225,270)
        ctx.move_to(-225,220)
        ctx.line_to(-225,270)
        ctx.stroke()
        ctx.set_source_rgb(0.7,0.9,0)
        ctx.move_to(-width,-240)
        ctx.line_to(-280,-240)
        ctx.line_to(-280,200)
        ctx.line_to(-width,200)
        ctx.stroke()
        #zero
        ctx.rectangle(-2,216,4,40)
        ctx.set_source_rgb(0.7,0,0)
        ctx.fill()
        ctx.rectangle(-320,-2,50,4)
        ctx.set_source_rgb(0.7,0,0)
        ctx.fill()


    def drawface(self,cr,w,h):

        #right
        cr.set_source_rgb ( 1, 1, 0 )
        cr.move_to(120,-10)
        cr.line_to(200,-10)
        cr.line_to(200,0)
        cr.line_to(130,0)
        cr.line_to(130,30)
        cr.line_to(120,30)
        cr.set_source_rgb ( 0, 0, 0 )
        cr.fill()
        cr.close_path()
        cr.stroke()

        cr.set_source_rgb ( 1, 1, 0)
        cr.move_to(120,-10)
        cr.line_to(200,-10)
        cr.line_to(200,0)
        cr.line_to(130,0)
        cr.line_to(130,30)
        cr.line_to(120,30)
        cr.close_path()
        cr.stroke()

        #left
        cr.set_source_rgb ( 1, 1, 0 )
        cr.move_to(-120,-10)
        cr.line_to(-200,-10)
        cr.line_to(-200,0)
        cr.line_to(-130,0)
        cr.line_to(-130,30)
        cr.line_to(-120,30)
        cr.set_source_rgb ( 0, 0, 0 )
        cr.fill()
        cr.close_path()
        cr.stroke()

        cr.set_source_rgb ( 1, 1, 0 )
        cr.move_to(-120,-10)
        cr.line_to(-200,-10)
        cr.line_to(-200,0)
        cr.line_to(-130,0)
        cr.line_to(-130,30)
        cr.line_to(-120,30)
        cr.close_path()
        cr.stroke()


        cr.set_source_rgb(0,0.9,0)
        cr.move_to(-13,0)
        cr.line_to(-110,0)
        cr.move_to(13,0)
        cr.line_to(110,0)
        cr.move_to(0,13)
        cr.line_to(-0,170)
        cr.move_to(0,-13)
        cr.line_to(0,-170)
        cr.stroke()
        #center
        cr.set_source_rgb(0.7, 0.9, 0.0)
        cr.set_line_width(3)
        cr.arc(0, 0, 10, 0, 2*math.pi)
        cr.stroke_preserve()

def run( Widget ):
    window = gtk.Window( )
    window.connect( "delete-event", gtk.main_quit )
    window.set_size_request ( 700, 600 )
    widget = Widget( )
    widget.show( )
    window.add( widget )
    window.present( )
    gtk.main( )



def getState(res):
    global roll,pitch,yaw,depth,x,y
    pose=res.pose.pose
    tmp=(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
    ang=euler_from_quaternion(tmp)

    x=pose.position.x
    y=pose.position.y
    depth=pose.position.z
    roll=ang[0]
    pitch=ang[1]
    

    yaw_tmp = ang[2]*180/math.pi

    yaw = min([yaw_tmp-360,360-yaw_tmp,360+yaw_tmp,yaw_tmp],key=abs)
    #print yaw

def processIMU_message(imuMsg):
    global roll,pitch,yaw,depth
    roll = imuMsg.orientation.x
    pitch = imuMsg.orientation.y
    yaw =  imuMsg.orientation.z
    omg = imuMsg.orientation.w
    roll,pitch,yaw = euler_from_quaternion((float(roll),float(pitch),float(yaw),float(omg)))
    yaw = yaw*180/math.pi
    if yaw > 0:
        yaw -= 360
    print "%.7f %.7f %.7f"%(roll,pitch,yaw)

def Getdepth(data):
    global depth
    depth = data.data

def GetJoy(data):
    global roll,pitch,yaw,depth,posex,posey
    #yaw = 4*data.axes[0]
    #pitch = 2*data.axes[1]
    #depth =4* data.axes[2]
   # roll = 2*data.axes[3]
    posex -= data.axes[4]
    posey -= data.axes[5]

#def start_display():
#    rospy.Subscriber('microstrain/data', Imu,processIMU_message)
#    rospy.Subscriber('altimeter/depth', Float64,Getdepth)
    #rospy.Subscriber('baro/data', Float64,Getdepth)
    #rospy.Subscriber('joy', Joy,GetJoy)
#    run( MyStuff )

rospy.init_node("node")
#rospy.Subscriber('gx4_45_imu/data', Imu,processIMU_message)
#rospy.Subscriber('altimeter/depth', Float64,Getdepth)
#rospy.Subscriber('baro/data', Float64,Getdepth)
rospy.Subscriber('/auv/state', Odometry, getState)

rospy.Subscriber('joy', Joy,GetJoy)
run( MyStuff )
