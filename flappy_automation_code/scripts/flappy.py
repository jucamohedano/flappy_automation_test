#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from matplotlib import pyplot as plt
from states import traverse

# constants and global variables
FPS = 30.
T = 1.0/FPS
bird_position = np.array([0., 0.], np.float32)
X_THRES=1.7
Y_TOP_THRES=1.7
Y_BOTTOM_THRES=-0.6#-1.1
accum_points = np.zeros((2,9), dtype=np.float32)
STATE=''
SCAN_STATE='WAITING'
TRAVERSE_STATE = 'WAITING'
y_coord_hole = None
desired_acc = np.array([0,0], np.float32)
current_vel = np.array([0,0], np.float32)
e_prev = 0.0
e_sum = 0.0
goal = 0
semaphore = 0
TOTAL_POINTS=1500

# Publisher for sending acceleration commands to flappy bird
pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)

def initNode():
    # Here we initialize our node running the automation code
    rospy.init_node('flappy_automation_code', anonymous=True)
    
    # Subscribe to topics for velocity and laser scan from Flappy Bird game
    rospy.Subscriber("/flappy_vel", Vector3, velCallback)
    rospy.Subscriber("/flappy_laser_scan", LaserScan, laserScanCallback)

    # Ros spin to prevent program from exiting
    rospy.spin()

def velCallback(msg):
    # msg has the format of geometry_msgs::Vector3
    # update bird position
    global bird_position, current_vel
    current_vel[0] = msg.x
    current_vel[1] = msg.y
    bird_position[0] += msg.x * T
    bird_position[1] += msg.y * T

    
    pub_acc_cmd.publish(Vector3((desired_acc[0]-current_vel[0]), (desired_acc[1]-current_vel[1]), 0))
    # pub_acc_cmd.publish(Vector3(desired_acc[0], desired_acc[1], 0))

def laserScanCallback(msg):
    # msg has the format of sensor_msgs::LaserScan
    # print laser angle and range
    inc = msg.angle_increment
    angles = np.linspace(msg.angle_min, msg.angle_max, 9)
    ranges = np.array(msg.ranges, dtype=np.float32())
    
    dist = np.array([bird_position[0] + np.cos(angles)*ranges, 
                     bird_position[1] + np.sin(angles)*ranges], 
                     np.float32)
    global STATE
    # print(STATE, bird_position)
    if STATE == '':
        STATE='GET_CLOSE'
    state_machine(dist)

def state_machine(dist):
    if STATE == 'GET_CLOSE':
        get_close(dist)
    elif STATE == 'SCAN_SEARCH':
        out = scan_search(dist)
    elif STATE == 'TRAVERSE':
        traverse()
        


#######################################################################
###################### STATES AND SUBSTATE MACHINE ####################
#######################################################################

def get_close(dist):
    global STATE
    result = pid_controller(4.4, 1, 0.5, 0, x=True)
    if result:
        reset_errors()
        STATE='SCAN_SEARCH'


def scan_search(dist):
    global STATE, accum_points

    # collect laser rages points while scaning
    if accum_points.shape[1] >= TOTAL_POINTS:
        global y_coord_hole
        hist = get_histogram(accum_points[1])
        idx = np.argmin(hist[0])
        y_coord_hole=(hist[1][idx] + hist[1][idx+1])/2.0        
        del accum_points # reset accum_points
        accum_points = dist
        STATE = 'TRAVERSE'
    else:
        scan()
        dist = np.where(dist[0] <= (bird_position[0]+X_THRES), dist, 0.0)
        if np.count_nonzero(dist) > 10:
            accum_points = np.hstack((accum_points, dist))

# use histogram to find the empty spot
def get_histogram(dataset):
    return np.histogram(dataset, 15, (Y_BOTTOM_THRES, Y_TOP_THRES))

# substate machine to perform scaning and find empty space
def scan():
    global SCAN_STATE
    if SCAN_STATE == 'WAITING':
        SCAN_STATE = 'GO_DOWN'
    elif SCAN_STATE == 'GO_DOWN':
        print('GO_DOWN')
        if go_down():
            SCAN_STATE = 'GO_UP'
    elif SCAN_STATE == 'GO_UP':
        print('GO_UP')
        if go_up():
            SCAN_STATE = 'WAITING'
    return


def go_down():
    # result = pid_controller(-0.65, 1.5, 0, 0)
    result = pid_controller(-0.6, 2.0, 2.1, 0.08)
    if result:
        reset_vars()
        return True
    return result

def go_up():
    # result = pid_controller(0.8, 1.5, 0, 0.)
    result = pid_controller(0.8, 2.0, 2.1, 0.08)
    if result:
        reset_vars()
        return True
    return result

# substate machine to traverse asteroid field
def traverse():
    global TRAVERSE_STATE
    if TRAVERSE_STATE == 'WAITING':
        TRAVERSE_STATE = 'GO_TO_SPACE'
    elif TRAVERSE_STATE == 'GO_TO_SPACE':
        if go_to_space():
            TRAVERSE_STATE = 'TRASPASS'
    elif TRAVERSE_STATE == 'TRASPASS':
        traspass()
    return

# goal in y direction to empty spot in the asteroid field
def go_to_space():
    result = pid_controller(y_coord_hole, 2.0, 1.8, 0.08)
    if result:
        reset_errors()
        return True

# cross asteroid field, goal in x direction
def traspass():
    global goal, semaphore
    if semaphore == 0:
        semaphore=1
        goal = bird_position[0]+1.9

    result = pid_controller(goal, 2.0, 1., 0, x=True)

    if result:
        reset_errors()
        reset_vars()
    return result

def reset_errors():
    global e_sum, e_prev
    e_sum = 0
    e_prev = 0

def reset_vars():
    global semaphore, SCAN_STATE, TRAVERSE_STATE, y_coord_hole, STATE
    semaphore = 0
    SCAN_STATE='WAITING'
    TRAVERSE_STATE = 'WAITING'
    y_coord_hole = None
    STATE='SCAN_SEARCH'

# PID controller
def pid_controller(g, K_P, K_D, K_I, x=False, min_error=1e-2):
    global e_sum, e_prev
    bird_pose=0

    if x == True:
        bird_pose=bird_position[0]     
    else:
        bird_pose=bird_position[1]
    

    e = g - bird_pose
    e_sum = e_sum + e * T
    de_dt = (e - e_prev) / T
    u = K_P * e + K_I * e_sum + K_D * de_dt

    if x:
        desired_acc[0] = u
    else:
        desired_acc[1] = u
    
    print('error={}, u={}'.format(abs(abs(g) - abs(bird_pose)), u))
    e_prev = e
    if abs(abs(g) - abs(bird_pose)) < min_error:
        return True
    else:
        return False


if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass
