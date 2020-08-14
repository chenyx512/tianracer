#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped


FILLER_VALUE = 4.0
def get_range(data, angle, deg=True):
    if deg:
        angle = np.deg2rad(angle)
    # on TianRacer, the front of the Lidar is the rear of the car
    angle = angle + np.pi
    if angle > np.pi:
        angle -= np.pi * 2
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    if dis < data.range_min or dis > data.range_max:
        dis = FILLER_VALUE
    return dis


def wall_following_callback(data):
    """
    Implements simple wall following at
    https://linklab-uva.github.io/autonomousracing/assets/files/assgn4-print.pdf
    """
    # the angle between the two laser rays
    THETA = np.pi / 180 * 30
    # target distance from wall
    TARGET_DIS = 0.5
    # the distance to project the car forward
    LOOK_AHEAD_DIS = 0.7
    P = 0.3

    # naming convention according to above pdf
    b = get_range(data, -90)
    a = get_range(data, -90 + THETA)
    # print(f"a{a:1.1f} b{b:1.1f}")
    alpha = np.arctan((a * np.cos(THETA) - b) / (a * np.sin(THETA)))
    AB = b * np.cos(alpha)
    projected_dis = AB + LOOK_AHEAD_DIS * np.sin(alpha)
    error = TARGET_DIS - projected_dis
    steering_angle = P * error

    front_dis = get_range(data, 0)
    speed = 0.5
    angle_filter = steering_angle
    drive_msg = AckermannDrive(steering_angle=steering_angle, speed=speed)
    # print(f"angle {np.rad2deg(steering_angle):2.1f}")
    drive_pub.publish(drive_msg)


rospy.init_node("techx")

scan_sub = rospy.Subscriber('/scan', LaserScan, wall_following_callback)

drive_pub = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=1)

rospy.spin()
