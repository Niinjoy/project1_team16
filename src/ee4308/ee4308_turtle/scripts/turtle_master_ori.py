#!/usr/bin/env python
# pylint: disable=import-error
import roslib, rospy, rospkg
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Bool, Empty
from math import pi, sqrt, sin, cos, floor
from numpy import nan

from ee4308_turtle.msg import EE4308MsgMotion
from turtle_constants import CLOSE_ENOUGH_SQ, COST_MAP_FREE, COST_MAP_UNK, TARGET_SEPARATION, PATH_PLANNER, COST_FUNCTION
import path_planners
import sys

# ================================= PARAMETERS ========================================== 
if PATH_PLANNER == "A*":
    path_planner = path_planners.a_star
else:
    path_planner = path_planners.theta_star
ITERATION_PERIOD = 0.1

# ================================= CONSTANTS ==========================================        

# ================================= GLOBALS ==========================================        
# some globals not listed -- only the ones required in service functions are listed
num_i = 0
num_j = 0
x_min = 0.
y_min = 0.
cell_size = 0.
using_map = False

# =============================== SUBSCRIBERS =========================================  
def subscribe_motion(msg):
    global msg_motion
    msg_motion = msg

def subscribe_map(msg):
    if using_map:
        return
    global msg_map
    msg_map = msg
    
# =============================== SERVICE =========================================  
def planner2topic(path, msg):
    # path is list of integers of turning points from path planner: e.g. [32,123,412]
    # msg is the msg to publish, which is a Path() instance
    poses = []
    
    for idx in path:
        i = idx[0]
        j = idx[1]
        
        pose = PoseStamped() # create new pose
        position = pose.pose.position # point to position
        position.x = x_min + (0.5+i)*cell_size # convert i to x, #0.5 added for rviz
        position.y = y_min + (0.5+j)*cell_size # convert j to y, #0.5 added for rviz
        position.z = 0.18 # to see it float above the rviz, purely for visualisation
        poses.append(pose)

    msg.poses = poses
    
def x2i(x):
    return int(round(float(x - x_min) / cell_size))
def y2j(y):
    return int(round(float(y - y_min) / cell_size))
def i2x(i):
    return x_min + i*cell_size
def j2y(j):
    return y_min + j*cell_size

def plan(robot_x, robot_y, goal_x, goal_y):
    global using_map
    # convert x, y, coordinates to i, j to read occupancy grid in path planner
    robot_i = x2i(robot_x)
    robot_j = y2j(robot_y)
    goal_i = x2i(goal_x)
    goal_j = y2j(goal_y)
    
    # plan using chosen planner
    using_map = True
    path_planner(robot_i, robot_j, goal_i, goal_j, msg_map.data)
    using_map = False
    # path returned is reversed (goal at the start, robot position at the end of lists)

def is_free(cost_map_value):
    return cost_map_value == COST_MAP_FREE or cost_map_value == COST_MAP_UNK    
# ================================ BEGIN ===========================================
def master(goals=[]):
    # ---------------------------------- INITS ----------------------------------------------
    
                
    # init node
    rospy.init_node('turtle_master')
    
    # Set the labels below to refer to the global namespace (i.e., global variables)
    # global is required for writing to global variables. For reading, it is not necessary
    global msg_map, msg_motion
    
    # Initialise global vars
    msg_motion = None
    msg_map = None

    # Subscribers
    rospy.Subscriber('/turtle/motion', EE4308MsgMotion, subscribe_motion, queue_size=1)
    rospy.Subscriber('/turtle/map', OccupancyGrid, subscribe_map, queue_size=1)
    
    # Publishers
    # send reset to calibrate gyro and reset odom
    rospy.Publisher('/turtle/reset', Empty, latch=True, queue_size=1).publish(Empty())
    
    pub_path = rospy.Publisher('/turtle/path', Path, latch=True, queue_size=1) # for rviz
    pub_target = rospy.Publisher('/turtle/target', PointStamped, latch=True, queue_size=1)
    pub_stop = rospy.Publisher('/turtle/stop', Bool, latch=True, queue_size=1)
    msg_stop = Bool()
    msg_stop.data = False
    pub_stop.publish(msg_stop) # ping to others it is ready
    
    # Wait for Subscribers to receive data.
    # ~ note imu will not publish if you press Ctrl+R on Gazebo. Use Ctrl+Shift+R instead
    print ("[ RESET] Calibrating Gyro and Resetting Odometry...")
    while (msg_motion is None or msg_map is None or rospy.get_time() == 0) and not rospy.is_shutdown():
        pass
        
    if rospy.is_shutdown():
        return
        
    # inits
    global num_i, num_j, cell_size, x_min, y_min, using_map
    tmp = msg_map.info
    num_j = tmp.width
    num_i = tmp.height
    cell_size = tmp.resolution
    tmp  = tmp.origin.position
    x_min = tmp.x
    y_min = tmp.y
    using_map = False
    
    path_planners.init_module(num_i, num_j)
    
    msg_path = Path();
    msg_path.header.frame_id = "map"
    
    msg_target = PointStamped()
    msg_target.header.frame_id = "map"
    msg_target.point.z = 0.18
    msg_target_position = msg_target.point
    
    # init goal position and index
    goal_idx = 0
    num_goals = len(goals)
    goal = goals[goal_idx]
    goal_x = goal[0]
    goal_y = goal[1]
    
    turnpt_x = goal_x
    turnpt_y = goal_y
    turnpt_idx = -1
    
    target_x = msg_motion.x
    target_y = msg_motion.y
    target_idx = -1
    num_targets = 1
    Dx = 0.
    Dy = 0.
    
    need_path = True
    need_trajectory = True
    
    print('=== [MASTER] Initialised ===')
    t = rospy.get_time()
    run_start = t
    while not rospy.is_shutdown():
        if rospy.get_time() > t:
            # check path if there is overlap
            using_map = True
            for k in path_planners.path_full: # for all cells in the full path...
                if not is_free(msg_map.data[k]): # if is occupied / inflated...
                    need_path = True # request a new path
                    print('[MASTER] Path intersects inf/occ cells, new path requested')
                    break
            using_map = False
        
            # # if there is no urgent need to replan
            ########
            
            # check if close enough to target
            Di = target_x - msg_motion.x
            Dj = target_y - msg_motion.y
            if Di*Di + Dj*Dj <= CLOSE_ENOUGH_SQ:
                # target reached
                target_idx += 1
                if target_idx < num_targets:
                    # still have targets remaining
                    target_x += Dx
                    target_y += Dy
                    
                    # publish new target
                    msg_target_position.x = target_x
                    msg_target_position.y = target_y
                    pub_target.publish(msg_target)
                else:
                    # no more targets remaining (i.e., reached turning point / goal)
                    turnpt_idx -= 1
                    if turnpt_idx >= 0:
                        # turning points remaining
                        turnpt = msg_path.poses[turnpt_idx].pose.position            
                        turnpt_x = turnpt.x
                        turnpt_y = turnpt.y
                        
                        # generate new targets (trajectory)
                        need_trajectory = True
                    else:
                        # no more turning points remaining, reached goal
                        goal_idx += 1
                        
                        # break if no more goals
                        if goal_idx == num_goals:
                            print("[MASTER] Final goal ({}, {}) reached!".format(goal_x, goal_y))
                            break
                        
                        print("[MASTER] Goal ({}, {}) reached, new path requested".format(goal_x, goal_y))
                        # get the next goal
                        goal = goals[goal_idx]
                        goal_x = goal[0]
                        goal_y = goal[1]
                        
                        # generate new path and targets (trajectory)
                        need_path = True
            
            if need_path:
                
                # replan the path
                plan(msg_motion.x, msg_motion.y, goal_x, goal_y) # given the logic, not possible to return a path with 1 idx (on the goal)
                print('[MASTER] Path Found')
                if not path_planners.path_pts:
                    print('[MASTER] No path found (robot/goal in occupied/inflation cell?)')
                    t += ITERATION_PERIOD # check in next iteration
                    
                    # publish as a fail safe, so it doesn't get trapped at a target point                    
                    
                    continue
                
                # convert to appropriate data and publish for visualisation in rviz
                msg_path.header.seq += 1
                planner2topic(path_planners.path_pts, msg_path)
                pub_path.publish(msg_path)
                    
                # get the first turning point (second point in path)
                turnpt_idx = len(path_planners.path_pts) - 2
                turnpt = msg_path.poses[turnpt_idx].pose.position
                turnpt_x = turnpt.x
                turnpt_y = turnpt.y
                
                need_path = False
                need_trajectory = True
                
            if need_trajectory:
                # generate points
                turnpt = msg_path.poses[turnpt_idx+1].pose.position # the cell which the robot is on; reused variable turnpt
                target_x = turnpt.x
                target_y = turnpt.y
                Dx = turnpt_x - target_x
                Dy = turnpt_y - target_y
                num_targets = sqrt(Dx*Dx + Dy*Dy) / TARGET_SEPARATION # find number of points
                Dx /= num_targets
                Dy /= num_targets 
                num_targets = int(floor(num_targets))
                # bypass robot position due to overshooting position while travelling
                target_idx = 1
                target_x += Dx * target_idx
                target_y += Dy * target_idx
                
                # publish new target
                msg_target_position.x = target_x
                msg_target_position.y = target_y
                pub_target.publish(msg_target)
                
                # switch to previous state
                need_trajectory = False
            ########
            
            et = (rospy.get_time() - t)
            if et > ITERATION_PERIOD:
                print('[MASTER] {}ms OVERSHOOT'.format(int(et*1000)))
            t += ITERATION_PERIOD
    
    msg_stop.data = True
    pub_stop.publish(msg_stop)
    rospy.sleep(1.) # sleep 1 second for topic to latch
    print('[MASTER] {}ms SECONDS ELAPSED'.format(rospy.get_time() - run_start))
    
        
if __name__ == '__main__':      
    try: 
        # parse goals
        if len(sys.argv) > 1:
            goals = sys.argv[1]
            goals = goals.split('|')
            for i in xrange(len(goals)):
                tmp = goals[i].split(',')
                tmp[0] = float(tmp[0])
                tmp[1] = float(tmp[1])
                goals[i] = tmp
            
            master(goals)
        else:
            master()
    except rospy.ROSInterruptException:
        pass
    print('=== [MASTER] Terminated ===')

