from argparse import ArgumentParser

# Note: all distances are measures in millimeters and angles in radians (unless stated otherwise)

# Fixed parameters
ROBOT_L = 121  # distance between robot wheels
ROBOT_r = 28  # radius of robots wheels
LIN_VEL = 200  # stable velocity for linear motion (mm/s)

# command line arguments
__parser = ArgumentParser()

__parser.add_argument("-l", "--length", help="Length of the linear trayectory (mm)", type=float, default=-1)  # 1000
__parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory (mm)", type=float, default=-1)  # 200
__parser.add_argument("-a", "--radioA", help="Radio 'a' to perform the Bicy-trajectory (mm)", type=float, default=-1)  # 100
__parser.add_argument("-r", "--distR", help="Distance 'r' to perform the Bicy-trajectory (mm)", type=float, default=-1)  # 500
__parser.add_argument("-f", "--log", help="Log odometry into a file", default=False)

__parser.add_argument("-nw", "--noWait", help="Disable odometry wait", action="store_true")
__parser.add_argument("-no", "--noOdometry", help="Don't use odometry for movement ", action="store_true")
__parser.add_argument("-e", "--exact", help="Use the exact method for odometry ", action="store_true")
__parser.add_argument("-np", "--noPlot", help="Don't show a plot with the values", action="store_true")

__args = __parser.parse_args()

for __key, __value in vars(__args).items():
    globals()[__key] = __value
