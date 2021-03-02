#!/usr/bin/python
# -*- coding: UTF-8 -*-
import brickpi3 # import the BrickPi3 drivers
import time     # import the time library for the sleep function
import sys
import numpy as np

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

L = 121 # mm
r = 28 # mm

class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

######## UNCOMMENT and FILL UP all you think is necessary (following the suggested scheme) ########

        # Robot construction parameters
        #self.R = ??
        #self.L = ??
        #self. ...

        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()

        # Configure sensors, for example a touch sensor.
        #self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        # reset encoder B and C (or all the motors you are using)
        self.BP.offset_motor_encoder(self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B))
        self.BP.offset_motor_encoder(self.BP.PORT_C, self.BP.get_motor_encoder(self.BP.PORT_C))

        ##################################################
        # odometry shared memory values
        self.x = Value('d',0.0)
        self.y = Value('d',0.0)
        self.th = Value('d',0.0)
        self.finished = Value('b',1) # boolean to show if odometry updates are finished

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()
        #self.lock_odometry.acquire()
        #print('hello world', i)
        #self.lock_odometry.release()

        # odometry update period --> UPDATE value!
        self.P = 0.25



    def setSpeed(self, v,w):
        """ To be filled - These is all dummy sample code """
        print("setting speed to %.2f %.2f" % (v, w))

        # compute the speed that should be set in each motor ...

        vr = v/r
        wl2r = w*L/2/r

        wd = vr + wl2r
        wi = vr - wl2r

        speedDPS_left = np.rad2deg(wi)
        speedDPS_right = np.rad2deg(wd)
        self.BP.set_motor_dps(self.BP.PORT_B, speedDPS_left)
        self.BP.set_motor_dps(self.BP.PORT_C, speedDPS_right)


    def readSpeed(self):
        """ To be filled"""

        return 0,0

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        return self.x.value, self.y.value, self.th.value

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=()) #additional_params?))
        self.p.start()
        print("PID: ", self.p.pid)

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self): #, additional_params?):
        """ To be filled ...  """

        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.clock()

            # compute updates

            ######## UPDATE FROM HERE with your code (following the suggested scheme) ########
            
            #print("Dummy update of odometry ...., X=  %.2f" %(self.x.value) )


            try:
                # Each of the following BP.get_motor_encoder functions returns the encoder value
                # (what we want to store).
                sys.stdout.write("Reading encoder values .... \n")
                encoderLeft = self.BP.get_motor_encoder(self.BP.PORT_B)
                encoderRight = self.BP.get_motor_encoder(self.BP.PORT_C)
                self.BP.offset_motor_encoder(self.BP.PORT_B, encoderLeft)
                self.BP.offset_motor_encoder(self.BP.PORT_C, encoderRight)
            except IOError as error:
                print(error)
            
            
            encoderLeft = np.deg2rad(encoderLeft)
            encoderRight = np.deg2rad(encoderRight)
            
            wi = encoderLeft / self.P
            wd = encoderRight / self.P
            
            v = r * (wd + wi) / 2
            w = r * (wd - wi) / L
            
            if w == 0:
            	deltaX = v * self.P
            	deltaY = 0
            	deltaTh = 0
            else:
            	R = v/w
            	deltaTh = w * self.P
            	deltaX = R * np.sin(deltaTh)
            	deltaY = R * (1-np.cos(deltaTh))
            	
            
            # update odometry uses values that require mutex
            # (they are declared as value, so lock is implicitly done for atomic operations, BUT =+ is NOT atomic)
            	
            # to "lock" a whole set of operations, we can use a "mutex"
            with self.lock_odometry:
	            self.x.value += np.cos(self.th.value)*deltaX - np.sin(self.th.value)*deltaY
	            self.y.value += np.sin(self.th.value)*deltaX + np.cos(self.th.value)*deltaY
	            self.th.value += deltaTh

            sys.stdout.write("Dummy update of odometry ...., X=  %.4f, \
                Y=  %.4f, th=  %.4f \n" %(self.x.value, self.y.value, np.rad2deg(self.th.value)) )


            # save LOG
            # Need to decide when to store a log with the updated odometry ...

            ######## UPDATE UNTIL HERE with your code ########


            tEnd = time.clock()
            time.sleep(self.P - (tEnd-tIni))

        #print("Stopping odometry ... X= %d" %(self.x.value))
        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" %(self.x.value, self.y.value, self.th.value))


    # Stop the odometry thread.
    def stopOdometry(self):
        self.finished.value = True
        self.BP.reset_all()

