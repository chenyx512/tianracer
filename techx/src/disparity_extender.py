#! /usr/bin/env python3

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped


def _get_yaw(data):
    w = data.w
    x = -data.z
    y = data.x
    z = -data.y

    yaw = -math.atan2(2.0 * (w * x + y * z), w * w - x * x - y * y + z * z)

    return yaw


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


target_x = np.nan
target_y = np.nan
cur_x = np.nan
cur_y = np.nan
cur_theta = np.nan

DISPARITY_DIF = 0.3
CAR_WIDTH = 0.3
angle_P = 1 / 30 * 0.41
speed_P = 1 / 3.0
MIN_SPEED = 0.7
MAX_SPEED = 2


def disparity_extender_callback(data):
    """
    Implements disparity extender at
    https://www.nathanotterness.com/2019/04/the-disparity-extender-algorithm-and.html
    """
    global cur_x, cur_y, cur_theta, target_x, target_y
    dis = []
    for angle in range(-90, 91):
        dis.append(get_range(data, angle))

    disparities = []  # tuples (min_dis, side_range)
    for i in range(len(dis)):
        if i == len(dis) - 1:
            continue
        if abs(dis[i] - dis[i + 1]) > DISPARITY_DIF:
            min_dis = min(dis[i], dis[i + 1])
            angle_range = math.ceil(
                math.degrees(math.atan(CAR_WIDTH / 2 / min_dis)))
            angle_range += 3

            side_range = range(i - angle_range + 1, i + 1) if dis[
                                                                  i + 1] == min_dis \
                else range(i + 1, i + 1 + angle_range)
            # print(f"found disparity at {i - 90} with angle_range {angle_range},"
            #       f"{side_range}")
            disparities.append((min_dis, side_range))

    for min_dis, side_range in disparities:
        for i in side_range:
            if i >= 0 and i < len(dis):
                dis[i] = min(dis[i], min_dis)

    max_index = np.argmax(dis)
    max_dis = dis[max_index]
    target_angle = max_index - 90
    if not np.isnan(cur_x) and not np.isnan(cur_theta) and not np.isnan(cur_theta):
        target_x = cur_x + np.cos(math.radians(target_angle) + cur_theta) * max_dis
        target_y = cur_y + np.sin(math.radians(target_angle) + cur_theta) * max_dis


def odom_callback(data):
    global cur_x, cur_y, cur_theta, target_x, target_y
    speed = 0
    steering_angle = 0
    position = data.pose.pose.position #now Pose
    cur_x = position.x
    cur_y = position.y

    orientation = data.pose.pose.orientation
    cur_theta = _get_yaw(orientation)

    if not np.isnan(target_x) and not np.isnan(target_y):
        target_theta = math.atan2(target_y - cur_y, target_x - cur_x)
        err_theta = target_theta - cur_theta
        if err_theta > np.pi:
            err_theta -=  np.pi
        elif err_theta < -np.pi:
            err_theta += np.pi
        steering_angle = math.degrees(err_theta) * angle_P
        speed = np.hypot(target_y - cur_y, target_x - cur_x) * speed_P
        speed = np.clip(speed, MIN_SPEED, MAX_SPEED)
        rospy.loginfo_throttle(1.0,
            f"target_angle {math.degrees(err_theta)} speed {speed:1.1f} "
            f"steering_angle {math.degrees(steering_angle):3.1f}")

    drive_msg = AckermannDrive(steering_angle=steering_angle, speed=speed)
    drive_pub.publish(drive_msg)


rospy.init_node("techx")

scan_sub = rospy.Subscriber('/scan', LaserScan, disparity_extender_callback)
odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)

drive_pub = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=1)

rospy.spin()
