#!/usr/bin/env python

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
    num_goals = len(goals)  # 目标点的数量
    goal = goals[goal_idx]  # 当前goal的值，在初始化的时候设定的是第一个值
    goal_x = goal[0]
    goal_y = goal[1]
    
    turnpt_x = goal_x  # 中间的转折点的x，在这里turnpt和goal是一样的
    turnpt_y = goal_y  # 
    turnpt_idx = -1
    
    target_x = msg_motion.x
    target_y = msg_motion.y
    target_idx = -1
    num_targets = 1
    Dx = 0.
    Dy = 0.
    
    need_path = True
    need_trajectory = True
    
    inf_count = 0

    print('=== [MASTER] Initialised ===')
    t = rospy.get_time()
    run_start = t
    while not rospy.is_shutdown():
        if rospy.get_time() > t:
            # check path if there is overlap
            using_map = True
            for k in path_planners.path_full: # for all cells in the full path...
                if not is_free(msg_map.data[k]): # if is occupied / inflated... 如果其在运动的时候路途中的某一个点是inflation或者occupied则需要再次规划路径
                    need_path = True # request a new path
                    print('[MASTER] Path intersects inf/occ cells, new path requested')
                    break

            # Dg = goal_x - msg_motion.x
            # Dg = goal_y - msg_motion.y
            # if not is_free(goals[goal_idx]) and (Dg*Dg + Dg*Dg <= 10*CLOSE_ENOUGH_SQ) :
            #     goal_idx += 1
            #     need_path = True # request a new path
            
            using_map = False
        
            # if there is no urgent need to replan
            
            ########
            
            # check if close enough to target 查看机器人当前位置和目标点是否足够接近
            Di = target_x - msg_motion.x
            Dj = target_y - msg_motion.y
            if Di*Di + Dj*Dj <= CLOSE_ENOUGH_SQ:
                # target reached 如果和target的距离足够近，则直接认为我们已经到达了这个traget，可以看看下一个target了
                target_idx += 1
                if target_idx < num_targets:
                    # still have targets remaining 说明对于这个trajectory，在这个target之后还有新的target没有完成，还需要继续运动
                    # 在第一次运行的时候由于还没有生成路径，因此在这里Dx和Dy都是0
                    # 利用在前面trajectory计算中得到的Dx和Dy计算出下一个target的坐标位置
                    # 由于目标点和inflation重合了，所以实际上无论怎么进行规划和运动，都不能达到这最后一个target xxxx
                    # ？？？同样的，对于最后一条路径，如果其在某一个target上被inflation堵住了，其一直无法运动，但是在规划里面Dx一直叠加导致在直线上直接叠加到了目标点？所以导致了最后终结
                    target_x += Dx
                    target_y += Dy
                    
                    # publish new target 在计算出下一个target以后将其作为信息进行发送
                    msg_target_position.x = target_x
                    msg_target_position.y = target_y
                    pub_target.publish(msg_target)
                    
                else:
                    # no more targets remaining (i.e., reached turning point / goal) 到达了中途所需要的点turning point或者已经到达了goal
                    turnpt_idx -= 1
                    # 在第一轮运行的时候 turnpt_idx本来就已经是-1了，在操作后变为-2
                    # 由于这个turnpt是倒序的，所以要一直往下减，每次到达一个turning point就要-1，在turning point为0的时候就要
                    if turnpt_idx >= 0:
                        # turning points remaining 说明中间还有turning point还没有通过
                        turnpt = msg_path.poses[turnpt_idx].pose.position            
                        turnpt_x = turnpt.x
                        turnpt_y = turnpt.y
                        
                        # generate new targets (trajectory) 需要生成新的trajectory
                        need_trajectory = True
                    else:
                        # no more turning points remaining, reached goal 所有的中途点都已经通过了，说明已经达到了我们需要的当前goals，因此让goal的下标+1寻找下一个goal
                        goal_idx += 1
                        
                        # break if no more goals 如果这个时候所有的goal都已经达到了，那么说明我们的规划已经结束了，循环也可以结束了
                        if goal_idx == num_goals:
                            # 同时判断在真实世界中是不是真的到了
                            # 计算当前点和当前目标点的距离
                            Dgx = goal_x - msg_motion.x
                            Dgy = goal_y - msg_motion.y
                            # 如果当前点的距离和goal足够接近（距离小于50cm） 判断确实到达了最终目标点
                            if Dgx*Dgx + Dgx*Dgx <= 25*CLOSE_ENOUGH_SQ:
                                print("[MASTER] Final goal ({}, {}) reached!".format(goal_x, goal_y))
                                # 否则重新设置goal，进行path plan
                                break # 这里是整个程序的出口
                            else:
                                goal_idx -= 1
                                
                                print("[MASTER] not reached the final goal, need to replan")
                                goal = goals[goal_idx]
                                goal_x = goal[0]
                                goal_y = goal[1]

                                need_path = True
                                continue # 直接跳过整个过程重新进行运算（不能打印后面的终止信息）
                            
                        
                        print("[MASTER] Goal ({}, {}) reached, new path requested".format(goal_x, goal_y))
                        # get the next goal 否则说明我们需要规划通往下一个goal的路径了
                        goal = goals[goal_idx]
                        goal_x = goal[0]
                        goal_y = goal[1]
                        
                        # generate new path and targets (trajectory) 需要规划新的path
                        need_path = True
            
            if need_path:
                
                # replan the path 由于需要新的path，因此调用plan函数重新计算一条路径
                plan(msg_motion.x, msg_motion.y, goal_x, goal_y) # given the logic, not possible to return a path with 1 idx (on the goal)
                print('[MASTER] Path Found')
                if not path_planners.path_pts:
                    print('[MASTER] No path found (robot/goal in occupied/inflation cell?)')
                    
                    # 加入一个计数器，如果连续10次检测到了本次goal在障碍物里面则直接跳过这个goal
                    inf_count += 1
                    print(inf_count)
                    if inf_count > 20:
                        
                        ri = x2i(msg_motion.x)
                        rj = y2j(msg_motion.y)
                        rk = ri*num_j + rj

                        # robot本身在occ/inf里面

                        if not is_free(msg_map.data[rk]):
                            msg_target_position.x = target_x - Dx
                            msg_target_position.y = target_y - Dy
                            pub_target.publish(msg_target)
                            print("[MASTER] Go back to the front target")

                        # goal在occ/inf里面
                        else: 
                            # 计算当前点和当前目标点的距离
                            Dgx = goal_x - msg_motion.x
                            Dgy = goal_y - msg_motion.y
                            
                            # 如果在同一个地方找不到路径超过20次，同时机器人已经和当前目标点非常接近
                            # （20个循环的检测中，这一goal均在得到的map的inflation中，我们则直接寻找下一个goal）
                            
                                
                            # 如果当前点和goal非常接近，直接认为已经到达了目标点了
                            if Dgx*Dgx + Dgx*Dgx <= 25*CLOSE_ENOUGH_SQ:
                                goal_idx += 1
                                print("[MASTER] The robot nearly reach the Goal ({}, {}) in inf/occ, new path requested".format(goal_x, goal_y))
                            # 反之如果难以到达当前的目标点，则考虑直接掠过，寻找下一个目标点
                            else:
                                msg_target_position.x = target_x - Dx
                                msg_target_position.y = target_y - Dy
                                pub_target.publish(msg_target)
                                print("[MASTER] Go back to the front target")
                                #print("[MASTER] Warning: The robot can't reach the Goal ({}, {}) in inf/occ, new path requested".format(goal_x, goal_y))
                    need_path = True
                        
                    t += ITERATION_PERIOD # check in next iteration
                    
                    # publish as a fail safe, so it doesn't get trapped at a target point                    
                    
                    continue

                # 当找到了一条新的路径，则此时直接清空计数
                inf_count = 0
                # convert to appropriate data and publish for visualisation in rviz
                msg_path.header.seq += 1
                planner2topic(path_planners.path_pts, msg_path)
                pub_path.publish(msg_path)
                    
                # get the first turning point (second point in path)  从我们得到的整条路径中得到第一个所规划出的第一个turning point点（一条路径应该会产生多个turning point点）
                # -2的原因是由于这里是倒序，而py的序号是从0开始的，因此是长度-1是第一个元素，在这里去掉起点因此是len-2
                turnpt_idx = len(path_planners.path_pts) - 2
                turnpt = msg_path.poses[turnpt_idx].pose.position
                turnpt_x = turnpt.x
                turnpt_y = turnpt.y
                # 到这里为止，由于我们已经规划过了path，在这里除非还有更新，否则不需要再调用生成path的函数，但是由于还没有规划路径，因此trajectory的参数仍为True
                need_path = False
                need_trajectory = True
                
            if need_trajectory:
                # generate points 根据我们在前面得到的turning point来规划我们所需要的轨迹
                turnpt = msg_path.poses[turnpt_idx+1].pose.position # the cell which the robot is on; reused variable turnpt 获取下一个turning point的信息
                # 将机器人要移动到turning point的设定为我们的target
                target_x = turnpt.x
                target_y = turnpt.y
                # 计算这一个turning point和下一个turning point（traget）的距离，这个距离会在每次循环到达下一个target中被调用，直到到达我们所需要的turning point才会在下一个trajectory计算中被重新计算
                Dx = turnpt_x - target_x
                Dy = turnpt_y - target_y
                # 根据我们计算出的绝对距离，对这个轨迹进行一个等距离的分割
                num_targets = sqrt(Dx*Dx + Dy*Dy) / TARGET_SEPARATION # find number of points
                # 分别获得每一段的距离
                Dx /= num_targets
                Dy /= num_targets 
                num_targets = int(floor(num_targets)) #将target的数量转换为int
                # bypass robot position due to overshooting position while travelling 初始化为第一个target，
                target_idx = 1
                target_x += Dx * target_idx
                target_y += Dy * target_idx
                
                # publish new target 将这个第一个target进行发布
                msg_target_position.x = target_x
                msg_target_position.y = target_y
                pub_target.publish(msg_target)
                
                # switch to previous state 由于已经完成了轨迹的规划，轨迹上的每一个点都已经得到了
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
        # 文件里面将所需要的goal读入
        if len(sys.argv) > 1:
            goals = sys.argv[1]
            goals = goals.split('|')
            for i in xrange(len(goals)):
                tmp = goals[i].split(',')
                tmp[0] = float(tmp[0])
                tmp[1] = float(tmp[1])
                goals[i] = tmp
            # 运行master的程序
            master(goals)
        else:
            master()
    except rospy.ROSInterruptException:
        pass
    print('=== [MASTER] Terminated ===')

