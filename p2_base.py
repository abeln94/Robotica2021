#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
from Robot import Robot


def main(args):
    try:
        if args.radioD < 0:
            print('d must be a positive value')
            exit(1)

        # Instantiate Odometry. Default value will be 0,0,0
        # robot = Robot(init_position=args.pos_ini)
        robot = Robot()

        print("X value at the beginning from main X= %.2f" %(robot.x.value))

        # 1. launch updateOdometry Process()
        robot.startOdometry()
        
        # giro
        #robot.setSpeed(0,np.deg2rad(45))
        #time.sleep(8)
        #robot.setSpeed(0,0)
        #input("next")
        
        # avance
        #robot.setSpeed(200,0)
        #time.sleep(2)
        #robot.setSpeed(0,0)
        #input("next")

        # 2. perform trajectory
        T = 150 # mm/s
        d = 200 # mm
        #robot.setSpeed(T,T/d)
        #time.sleep(np.pi*d/T)
        #robot.setSpeed(T,-T/d)
        #time.sleep(2*np.pi*d/T)
        #robot.setSpeed(T,T/d)
        #time.sleep(np.pi*d/T)
        #robot.setSpeed(0,0)
        
        input("next")
        
        a = 100 # mm
        r = 500 # mm
        alpha = np.arctan2(d-a,r)
        robot.setSpeed(T, -T/a)
        time.sleep((np.pi/2-alpha)*a/T)
        robot.setSpeed(T,0)
        time.sleep(r/T)
        robot.setSpeed(T,-T/d)
        time.sleep((np.pi+2*alpha)*d/T)
        robot.setSpeed(T,0)
        time.sleep(r/T)
        robot.setSpeed(T, -T/a)
        time.sleep((np.pi/2-alpha)*a/T)


        # DUMMY CODE! delete when you have your own
        #robot.setSpeed(1,1)
        #print("Start : %s" % time.ctime())
        #time.sleep(3)
        #print("X value from main tmp %d" % robot.x.value)
        #time.sleep(3)
        #print("End : %s" % time.ctime())

        robot.lock_odometry.acquire()
        print("Odom values at main at the END: %.2f, %.2f, %.2f " % (robot.x.value, robot.y.value, robot.th.value))
        robot.lock_odometry.release()

        # PART 1:
        # robot.setSpeed()
        # until ...

        # PART 2:
        # robot.setSpeed()
        # until ...

        # ...



        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()


    except KeyboardInterrupt:
    # except the program gets interrupted by Ctrl+C on the keyboard.
    # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        robot.stopOdometry()

if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory (mm)",
                        type=float, default=40.0)
    args = parser.parse_args()

    main(args)



