"""
The Robot main class
"""
import os
import time
# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, RLock

import cv2
import numpy as np

from classes import Cfg
from classes.DeltaVal import DeltaVal
from classes.Map import Map, GRID
from classes.Periodic import Periodic
from functions.functions import norm_pi
from functions.get_color_blobs import get_blob, position_reached
from functions.get_image_match import match_images
from functions.math import sigmoid, logistic
from functions.simubot import simubot

try:
    import brickpi3  # import the BrickPi3 drivers
except ImportError:
    import simulation.simbrickpi3 as brickpi3

try:
    import picamera  # import the picamera
    from picamera.array import PiRGBArray
except ImportError:
    import simulation.simpicamera as picamera
    from simulation.simpicamera import PiRGBArray

Cfg.add_argument("-f", "--log", help="Log odometry into a file", default=False)
Cfg.add_argument("-u", "--updatePeriod", help="Update period in seconds", type=float, default=0.1)
Cfg.add_argument("-e", "--exact", help="Use the exact method for odometry", action="store_true")
Cfg.add_argument("-p", "--plot", help="Show a plot with the values", action="store_true")
Cfg.add_argument("-s", "--smoothness", help="Velocity update smoothness [0,1)", type=float, default=0.4)
Cfg.add_argument("-gyro", help="Use the gyroscope for rotation", action="store_true")
Cfg.add_argument("-mix", help="Mix the gyroscope and odometry for rotation", action="store_true")
Cfg.add_argument("-ball", help="Catch ball by alternate way", action="store_true")

# GYRO constants
GYRO_DEFAULT = 2371.1
GYRO2DEG = 0.24

FRICTION = 1.005


class Robot:
    def __init__(self, init_position=None):
        """
        Initialize basic robot params.
        Initialize Motors and Sensors according to the set up in your robot
        """
        if init_position is None:
            init_position = [0.0, 0.0, 0.0]

        ##################################################
        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()

        # Motors and sensors setup
        self.MOTOR_CLAW = self.BP.PORT_A
        self.MOTOR_LEFT = self.BP.PORT_B
        self.MOTOR_RIGHT = self.BP.PORT_C
        self.SENSOR_ULTRASONIC = self.BP.PORT_1
        self.SENSOR_BUTTON = self.BP.PORT_2
        self.SENSOR_LIGHT = self.BP.PORT_3
        self.SENSOR_GYRO = self.BP.PORT_4

        # Configure sensors, for example a touch sensor.
        self.BP.set_sensor_type(self.SENSOR_ULTRASONIC, self.BP.SENSOR_TYPE.EV3_ULTRASONIC_CM)
        # self.BP.set_sensor_type(self.SENSOR_ULTRASONIC, self.BP.SENSOR_TYPE.NXT_ULTRASONIC)
        self.BP.set_sensor_type(self.SENSOR_BUTTON, self.BP.SENSOR_TYPE.TOUCH)
        self.BP.set_sensor_type(self.SENSOR_LIGHT, self.BP.SENSOR_TYPE.NXT_LIGHT_ON)
        self.BP.set_sensor_type(self.SENSOR_GYRO, self.BP.SENSOR_TYPE.CUSTOM, [(self.BP.SENSOR_CUSTOM.PIN1_ADC)])

        # reset encoder of all motors
        for motor in (self.MOTOR_CLAW, self.MOTOR_LEFT, self.MOTOR_RIGHT):
            self.BP.offset_motor_encoder(motor, self.BP.get_motor_encoder(motor))

        # camera conf
        self.cam = picamera.PiCamera()
        self.cam.resolution = (Cfg.CAMERA_WIDTH, Cfg.CAMERA_HEIGHT)
        self.cam.framerate = 32

        ##################################################
        # Odometry

        self.p = None  # the odometry process

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = RLock()

        # odometry shared memory values
        self.x = Value('d', init_position[0], lock=self.lock_odometry)
        self.y = Value('d', init_position[1], lock=self.lock_odometry)
        self.th = Value('d', init_position[2], lock=self.lock_odometry)
        self.finished = Value('b', True, lock=self.lock_odometry)  # boolean to show if odometry updates are finished

        # odometry command values
        self.marker_x = Value('d', -999.0)
        self.marker_y = Value('d', -999.0)
        self.marker_th = Value('d', -999.0)
        self.marker_now = Value('b', False)

    def setSpeed(self, v, w):
        """
        Sets the speed of the robot to v linear motion (mm/s) and w angular motion (rad/s)
        :param v: linear velocity
        :param w: angular velocity
        """
        print("Setting speed to {:.2f} {:.2f}".format(v, w))

        # compute the speed that should be set in each motor ...
        wd = v / Cfg.ROBOT_r + w * Cfg.ROBOT_L / 2 / Cfg.ROBOT_r
        wi = v / Cfg.ROBOT_r - w * Cfg.ROBOT_L / 2 / Cfg.ROBOT_r

        self.BP.set_motor_dps(self.MOTOR_LEFT, np.rad2deg(wi))
        self.BP.set_motor_dps(self.MOTOR_RIGHT, np.rad2deg(wd))

    def readSpeed(self, readTime=0.1):
        """
        Reads the robot speed
        :param readTime: time to check in seconds (the bigger the better but also the longer)
        :return: the velocity as tuple (v,w) v linear velocity mm/s , w angular velocity rad/s
        """

        # read
        previ = self.BP.get_motor_encoder(self.MOTOR_LEFT)
        prevd = self.BP.get_motor_encoder(self.MOTOR_RIGHT)

        # wait
        time.sleep(readTime)

        # read
        posti = self.BP.get_motor_encoder(self.MOTOR_LEFT)
        postd = self.BP.get_motor_encoder(self.MOTOR_RIGHT)

        # calculate
        wi = np.deg2rad(posti - previ) / readTime
        wd = np.deg2rad(postd - prevd) / readTime

        v = (wi + wd) * Cfg.ROBOT_r / 2
        w = (wd - wi) * Cfg.ROBOT_r / Cfg.ROBOT_L

        return v, w

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        with self.lock_odometry:
            return self.x.value, self.y.value, self.th.value

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = True
        self.p = Process(target=self.updateOdometry)
        self.p.start()
        while self.finished.value: pass
        print("PID: ", self.p.pid)

    def updateOdometry(self):
        """ The odometry update process """

        # init map
        if Cfg.plot:
            map = Map()
            map.update(self.readOdometry())

        # init variables
        x, y, th = self.readOdometry()
        leftEncoder = DeltaVal(self.BP.get_motor_encoder(self.MOTOR_LEFT))
        rightEncoder = DeltaVal(self.BP.get_motor_encoder(self.MOTOR_RIGHT))

        if Cfg.log:
            fileName = Cfg.FOLDER_LOGS + Cfg.log
            os.makedirs(os.path.dirname(fileName), exist_ok=True)
            logFile = open(fileName, "w")
            logFile.write("Timestamp, X, Y, Theta\n")

        # ready
        self.finished.value = False

        # loop
        periodic = Periodic(Cfg.updatePeriod)
        while periodic(not self.finished.value):

            # get values
            dL = leftEncoder.update(self.BP.get_motor_encoder(self.MOTOR_LEFT))
            dR = rightEncoder.update(self.BP.get_motor_encoder(self.MOTOR_RIGHT))

            # compute updates
            if Cfg.exact:
                # exact, long
                dT = 1  # any value will work
                wL = np.deg2rad(dL) / dT
                wR = np.deg2rad(dR) / dT

                v = Cfg.ROBOT_r * (wL + wR) / 2
                w = Cfg.ROBOT_r * (wR - wL) / Cfg.ROBOT_L
                x, y, odo_th = simubot([v, w], np.array([x, y, th]), dT)

            else:
                # inexact, fast
                sR = np.deg2rad(dR) * Cfg.ROBOT_r
                sL = np.deg2rad(dL) * Cfg.ROBOT_r

                ds = (sL + sR) / 2
                dth = (sR - sL) / Cfg.ROBOT_L
                x += ds * np.cos(th + dth / 2)
                y += ds * np.sin(th + dth / 2)
                odo_th = norm_pi(th + dth)

            # update ang with gyro
            gyro_data = self.BP.get_sensor(self.SENSOR_GYRO)[0]
            gyro_speed = np.deg2rad((GYRO_DEFAULT - gyro_data) * GYRO2DEG)
            gyro_th = norm_pi(th + gyro_speed * periodic.delay)

            # final udpate
            if Cfg.gyro:
                th = gyro_th
            elif Cfg.mix:
                th = gyro_th \
                    if abs(gyro_speed) > np.deg2rad(20) \
                    else odo_th
            else:
                th = odo_th

            # detect marker
            if self.getLight() < 0.4 or self.marker_now.value:  # dark
                print("marker detected")
                if self.marker_x.value > -999: x = self.marker_x.value
                if self.marker_y.value > -999: y = self.marker_y.value
                if self.marker_th.value > -999: th = self.marker_th.value
                self.onMarker()

            # update
            with self.lock_odometry:
                self.x.value = x
                self.y.value = y
                self.th.value = th

            # display
            print("Updated odometry ... X={:.2f}, Y={:.2f}, th={:.2f}º, old_th={:.2f}º".format(x, y, np.rad2deg(th),
                                                                                               np.rad2deg(odo_th)))

            if Cfg.plot:
                map.update([x, y, th])

            # save LOG
            if Cfg.log:
                logFile.write("{}, {}, {}, {}\n".format(periodic.time, x, y, th))

            ######## UPDATE UNTIL HERE with your code ########

        print("Stopping odometry")
        if Cfg.log:
            logFile.close()

    def stopOdometry(self):
        """ Stop the odometry thread """
        self.finished.value = True
        if self.p is not None: self.p.join()
        self.BP.reset_all()
        self.cam.close()
        cv2.destroyAllWindows()

    def capture_image(self):
        """ Returns a BGR image taken at the moment """
        rawCapture = PiRGBArray(self.cam, size=(Cfg.CAMERA_WIDTH, Cfg.CAMERA_HEIGHT))
        self.cam.capture(rawCapture, format="bgr", use_video_port=True)
        return rawCapture.array

    def trackObject(self, targetPosition=(0.54, 0.165)):
        """
        Track one object with indicated color until the target size and centroid are reached
        :param targetPosition: on image target coordinates value of the blob's centroid
        """
        NOT_FOUND_WAIT = 20  # frames
        MOVEMENT_TIME = 0.1  # seconds
        ANGULAR_SPEED_LOST = np.deg2rad(30)  # angular speed when no blob found
        BACKTRACK_VELOCITY = 25  # mm/s

        # 0. Parameters
        notFoundCounter = 0
        found = False

        # 1. Loop running the tracking until target (centroid position and size) reached
        periodic = Periodic()
        while periodic(not found):

            # 1.1. search the most promising blob ..
            img = self.capture_image()
            position = get_blob(img)

            # 1.2. check the given position
            if position is not None:
                # 1.3 blob found, check its position for planning movement
                notFoundCounter = 0
                x, y = position
                print("found ball at x=", x, "y=", y)

                if y < targetPosition[1]:
                    # 1.4 target position reached, let's catch the ball
                    print("ball in position")
                    self.setSpeed(0, 0)  # stop moving
                    found = True  # stop loop immediately

                else:
                    # 1.4 angular movement to get a proper orientation to the target
                    angular_speed = sigmoid(x - targetPosition[0], 4) * Cfg.ANG_VEL
                    # 1.5 linear movement to get closer the target
                    linear_speed = logistic(y - targetPosition[1], 8, 0.30) * Cfg.LIN_VEL
                    if Cfg.ball:
                        if abs(angular_speed) > 0.2:
                            self.setSpeed(0, angular_speed)
                        else:
                            self.setSpeed(linear_speed, angular_speed)
                    else:
                        self.setSpeed(linear_speed, angular_speed)

            else:
                # 1.3 no blob found

                if position_reached(img):
                    # 1.4 target position reached, let's catch the ball
                    print("ball lost because of too near")
                    self.setSpeed(0, 0)  # stop moving
                    found = True  # stop loop immediately

                else:
                    if notFoundCounter > NOT_FOUND_WAIT:
                        # turn around until finding something similar to the target
                        print("not found, turn around")
                        self.setSpeed(0, ANGULAR_SPEED_LOST)
                    else:
                        # wait a bit
                        print("temporary lost")
                        notFoundCounter += 1
                        self.setSpeed(-BACKTRACK_VELOCITY, 0)

            if not found:
                time.sleep(MOVEMENT_TIME)

        # 2. Then catch the ball
        ANGLE = 90  # degrees
        TIME = 3  # seconds
        ADVANCE = 45  # mm (more or less)

        # sanity check
        if self.BP.get_motor_encoder(self.MOTOR_CLAW) > ANGLE / 2:
            print("attempting to use claw again")
            return

        # catch
        self.BP.set_motor_dps(self.MOTOR_CLAW, ANGLE / TIME)
        self.setSpeed(ADVANCE / TIME, 0)
        time.sleep(TIME)
        self.BP.set_motor_dps(self.MOTOR_CLAW, 0)
        self.setSpeed(0, 0)

    def rotate(self, th):
        """ makes the robot rotate 'th' radians, based on time """
        th = th * FRICTION
        self.setSpeed(0, Cfg.ANG_VEL * np.sign(th))
        time.sleep(abs(th) / Cfg.ANG_VEL)
        self.setSpeed(0, 0)

    def advance(self, dist):
        """ makes the robot advance 'dist' mm, based on time"""
        dist = dist * FRICTION
        self.setSpeed(Cfg.LIN_VEL * np.sign(dist), 0)
        time.sleep(abs(dist) / Cfg.LIN_VEL)
        self.setSpeed(0, 0)

    def go(self, x_goal, y_goal, radius=10):
        """
        Makes the robot move to a specific circle, based on the internal odometry
        :param x_goal: x coordinate of the circle's center
        :param y_goal: y coordinate of the circle's center
        :param radius: radius of the circle
        """
        self.lookAt(x_goal, y_goal)

        periodic = Periodic()

        while periodic():
            x, y, th = self.readOdometry()

            # calculate movement
            dx = x_goal - x
            dy = y_goal - y
            dth = norm_pi(np.arctan2(dy, dx) - th)
            dist = np.linalg.norm([dx, dy])

            if dist < radius:
                # inside the circle, stop
                self.setSpeed(0, 0)
                time.sleep(0.5)
                return

            # calculate velocity (all the constants were found by try&error)
            w = sigmoid(dth, 3) * Cfg.ANG_VEL
            v = logistic(dist, 0.01, 3) * Cfg.LIN_VEL

            # move
            self.setSpeed(v, w)

    def lookAt(self, x_goal, y_goal, arc=np.deg2rad(5)):
        """
        Rotates the robot so that it looks at some coordinates
        :param x_goal: x coordinate of the looked point (in world coordinates)
        :param y_goal: y coordinate of the looked point (in world coordinates)
        :param arc: threshold of the destination rotation (rad)
        """
        periodic = Periodic()

        while periodic():
            x, y, th = self.readOdometry()

            # calculate angle
            dx = x_goal - x
            dy = y_goal - y
            dth = norm_pi(np.arctan2(dy, dx) - th)

            if abs(dth) < arc:
                # inside the arc, stop
                self.setSpeed(0, 0)
                time.sleep(0.5)
                return

            # calculate velocity (all the constants were found by try&error)
            w = sigmoid(dth, 3) * Cfg.ANG_VEL

            # move
            self.setSpeed(0, w)

    def getObstacleDistance(self):
        """
        Uses the ultrasound sensor to get the distance to the next obstacle in front of the robot
        :return: the distance of the obstacle in front of the robot in mm
        """
        # return distance
        return (self.BP.get_sensor(self.SENSOR_ULTRASONIC) + 5) * 10

    def detectObstacle(self, x_dest, y_dest):
        """
        Detect an obstacle between the current position and the destination (turns the robot)
        :param x_dest:
        :param y_dest:
        :return: true iff there is an obstacle
        """

        # turn to look
        self.lookAt(x_dest, y_dest)
        x, y, _ = self.readOdometry()

        # check if there is nothing in between
        dist = np.linalg.norm([x_dest - x, y_dest - y])
        return self.getObstacleDistance() < dist

    def getLight(self):
        """
        Uses the light sensor to get the amount of light
        :return: the amount of light from 0 (dark, no light) to 1 (bright, full light)
        """
        return 1 - self.BP.get_sensor(self.SENSOR_LIGHT) / 4000

    def detectImage(self, imgage_bgr):
        """
        Returns wheter the given image is detected or not into the current camera's capture,
        and if it is, returns the matched blobs's coordinates
        """
        # Return the result of invoking find_image
        return match_images(imgage_bgr, self.capture_image())

    def waitButtonPress(self):
        """ Waits until the button is pressed&released """
        periodic = Periodic()
        while periodic(not self.BP.get_sensor(self.SENSOR_BUTTON)): pass  # wait for press
        while periodic(self.BP.get_sensor(self.SENSOR_BUTTON)): pass  # wait for release

    def onMarker(self, x=-999, y=-999, th=-999, now=False):
        """
        Marks the values that the odometry will be replaced with when the black mark is detected
        Values not specified will be left untouched
        If 'now' is true the update is instant (without waiting for the marker)
        """
        self.marker_x.value = x
        self.marker_y.value = y
        self.marker_th.value = th
        self.marker_now.value = now

    def updateOdOnWall(self, ANG=60):
        """
        Makes the robot rotate until it is perpendicular to the wall in front
        :param ANG: angle used for finding the wall
        """
        ROTATION = 2.5
        # We assume robot looking a front wall
        ang, best_ang = -ANG, -ANG
        self.rotate(np.deg2rad(best_ang))
        best_dist = 2 * GRID
        while ang <= ANG:
            ang += ROTATION
            self.rotate(np.deg2rad(ROTATION))
            new_dist = 0
            for i in range(5):
                new_dist += self.getObstacleDistance() / 5

            if new_dist < best_dist:
                best_dist = new_dist
                best_ang = ang

        # self.onMarker(0, 0, new_th, True)
        self.rotate(-np.deg2rad(ANG - best_ang))
        return best_dist
