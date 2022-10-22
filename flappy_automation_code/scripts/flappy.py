#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

FPS = 30
SCALING = 0.01
MAX_ACC= 35.
target_vel = np.array([0,0], np.float32)
current_vel = np.array([0,0], np.float32)
x_vel = 0.
accum_y_distances = np.zeros((8,1), dtype=np.float32)
TOLERANCE = 0.15
dt=0.5

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
    # Example of publishing acceleration command on velocity velCallback
    
    global current_vel

    if not np.any(current_vel):
        current_vel = np.array([x_vel,0])

    direction = 1
    if target_vel[1] < 0:
        direction = -1        

    vel_cmd = abs(target_vel) - abs(current_vel)
    vel_cmd *= direction

    # print(target_vel[1], current_vel[1])
    # print('vel_cmd= {}'.format(vel_cmd[1]))
    # print('current velocity: {}'.format(current_vel))

    current_vel = vel_cmd

    # pub_acc_cmd.publish(Vector3(vel_cmd[0], vel_cmd[1], 0))


def laserScanCallback(msg):
    # msg has the format of sensor_msgs::LaserScan
    # print laser angle and range
    
    global target_vel, current_vel, x_vel
    
    inc = msg.angle_increment
    ranges = np.array(msg.ranges, dtype=np.float32()).reshape(9,)

    print('intensities={}'.format(msg.intensities))

    offset = 2.5
    formatted_ranges = np.array([
        ranges[0] if ranges[0] <= offset else np.nan,
        ranges[1] if ranges[1] <= offset else np.nan,
        ranges[2] if ranges[2] <= offset else np.nan,
        ranges[3] if ranges[3] <= offset else np.nan,
        ranges[5] if ranges[5] <= offset else np.nan,
        ranges[6] if ranges[6] <= offset else np.nan,
        ranges[7] if ranges[7] <= offset else np.nan,
        ranges[8] if ranges[8] <= offset else np.nan
    ])

    # y_distances = replace_empty_with_means(y_distances)
    formatted_ranges = replace_empty_with_means(formatted_ranges)

    y_distances = np.array([
        np.sin(inc*4)* formatted_ranges[0],# if ranges[0] <= offset else np.nan,
        np.sin(inc*3)* formatted_ranges[1],# if ranges[1] <= offset else np.nan,
        np.sin(inc*2)* formatted_ranges[2],# if ranges[2] <= offset else np.nan,
        np.sin(inc)  * formatted_ranges[3],# if ranges[3] <= offset else np.nan,
        np.sin(inc)  * formatted_ranges[4],# if ranges[4] <= offset else np.nan,
        np.sin(inc*2)* formatted_ranges[5],# if ranges[5] <= offset else np.nan,
        np.sin(inc*3)* formatted_ranges[6],# if ranges[6] <= offset else np.nan,
        np.sin(inc*4)* formatted_ranges[7] # if ranges[7] <= offset else np.nan     
    ])
    
    global accum_y_distances
    if accum_y_distances.shape[1] >= 8:
        del accum_y_distances
        accum_y_distances = y_distances
    else:
        accum_y_distances = np.hstack((accum_y_distances, y_distances))
        y_distances = np.mean(accum_y_distances, axis=1).reshape(8,1)
        # y_distances = np.subtract(accum_y_distances[-1], accum_y_distances[-2])
    print(y_distances)
    y_distances = 0.1/(y_distances**2)
    sum_all = np.sum(y_distances[:4])-np.sum(y_distances[4:])

    # check how many lasers pointing to inf
    num_lasers = np.where(ranges > msg.range_max-0.1)[0].shape[0]
    
    if not num_lasers == 7 and not np.isnan(sum_all):
        # print(MAX_ACC / sum_all)
        # if floor <= 0.05 and floor > 0.1:
        #     target_vel[1] = 0.
        # elif ceiling <= 0.5 and ceiling > 0.1:
        #     target_vel[1] = 0.
        # else:
        # if sum_all > -0.5 and sum_all < 1.0:
        #     target_vel[1] = 0.0
        # else:
        if sum_all > -0.01 and sum_all < 0.01:
            sum_all=0
        else:
            if ranges[4] < 2.0:
                target_vel[1] *= current_vel[1]*0.5
            else:
                target_vel[1] = sum_all
        
        print(sum_all)
    
    
# replace all the nan values with the means of each corresponding part (above/below the front laser)
def replace_empty_with_means(y_distances):
    a = y_distances[:4]
    b = y_distances[4:]

    a_ind = np.argwhere(np.isnan(a))
    b_ind = np.argwhere(np.isnan(b))
    a_mean = np.nanmean(a)
    b_mean = np.nanmean(b)

    a[a_ind] = a_mean
    b[b_ind] = b_mean
    concat = np.hstack((a, b))
    return concat.reshape(8,1)


def is_close_to_wall(y_distances):
    first = y_distances[0]
    last = y_distances[7]

    floor = np.cos(45)*first
    ceiling = np.cos(45)*last

    print('floot={} \t ceiling={}'.format(floor, ceiling))
    return floor, ceiling

    


if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass
