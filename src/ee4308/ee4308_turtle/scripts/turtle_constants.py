#!/usr/bin/env python
from math import pi, sqrt

# General
DEG2RAD = [i/180.0*pi for i in xrange(360)]
SQRT2 = sqrt(2.)
TWOPI = 2.*pi
HALFPI = 0.5*pi

# Turtle Mapper
L_OCC = 1 # log_odds for occupied cell, but scaled to an integer.
L_FREE = -L_OCC # assume log_odds for free cell is negative of log_odds for occupied cell
L_THRESH = 10 # <= L_THRESH means free; >= L_THRESH means occupied; in between is unknown. Scaled proportionally to L_OCC and L_FREE
L_MAX = 30 # limit possible values of log odds in occupancy grid to between -L_MAX and L_MAX. Scaled proportionally to L_OCC and L_FREE
COST_MAP_FREE = 101 # used for coloring and identifying free cells on a map
COST_MAP_OCC = 0 # used for coloring and identifying occupied cells on a map
COST_MAP_UNK = -1 # used for coloring and identifying unknown cells on a map
COST_MAP_INF = 60 # used for coloring and identifying inflated cells on a map
MAX_SCAN_RANGE = 3.5 # maximum scanning range of LIDAR

# Turtle Master
CLOSE_ENOUGH = 0.1 # threshold to consider close enough to target / turningpoint / goal (m)
CLOSE_ENOUGH_SQ = CLOSE_ENOUGH*CLOSE_ENOUGH # (m^2)
TARGET_SEPARATION = 0.05 # separation between targets for pure pursuit (m)
PATH_PLANNER = "THETA*" # "A*" or "THETA*"
COST_FUNCTION = "EUCLIDEAN" # "DIAGONAL" or "EUCLIDEAN"

# Turtle Motion
AXLE_TRACK = 0.16 #m
WHEEL_RADIUS = 0.033 #m
TRUST_ODM_V = 0.9 # gain between 0 and 1. Higher to trust odometry more than imu in forward velocity measurements
TRUST_ODM_W = 0.1 # gain between 0 and 1. Higher to trust odometry more than imu in angular velocity measurements
TRUST_IMU_V = 1. - TRUST_ODM_V
TRUST_IMU_W = 1. - TRUST_ODM_W
STRAIGHT_TOLERANCE = 5e-1 # rad/s. Tolerance to switch between motion models during high and low angular velocities
USE_ODOM = True # is the ground truth only in simulation

# Turtle Move
# constraints
MIN_V = 0.0 # minimum forward speed (magnitude) (m/s) to avoid steady-state errors in position
MAX_V = 0.22 # maximum forward speed (magnitude) (m/s)
MAX_W = 2.84 # maximum angular speed (magnitude) (rad/s)
MAX_DV = 0.05 # maximum change in forward speed (m/s) in one ITERATION_PERIOD
MAX_DW = 0.1 # maximum change in angular speed (rad/s) in one ITERATION_PERIOD
# TURN_THRESHOLD = pi / 3. # if more than TURN_THRESHOLD radians from target, forward speed is zero. Else, is a quadratic triangle function (rad)
# define PD controller gains
KP_V = 4.0# P gain for forward velocity
KP_W = 2.0 # P gain for angular velocity
KD_V = 8.0 # D gain for forward velocity
KD_W = 8.0 # D gain for angular velocity
DISABLE_MOVE = False
