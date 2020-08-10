#! /usr/bin/env python3

import rospy
import math
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


def disparity_extender_callback(data):
    """
    Implements disparity extender at
    https://www.nathanotterness.com/2019/04/the-disparity-extender-algorithm-and.html
    """
    DISPARITY_DIF = 0.3
    CAR_WIDTH = 0.5
    angle_P = 1 / 30 * 0.41
    speed_P = 1 / 12 * 7

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
            angle_range += 6

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
    steering_angle = target_angle * angle_P

    speed = get_range(data, 0) * speed_P
    if speed > 2: speed = 2
    if speed < 0.5: speed = 0.5
    print(f"target_angle {target_angle} speed {speed:1.1f} "
          f"steering_angle {math.degrees(steering_angle):3.1f}")

    drive_msg = AckermannDrive(steering_angle=steering_angle, speed=speed)
    # print(f"angle {np.rad2deg(steering_angle):2.1f}")
    drive_pub.publish(drive_msg)


rospy.init_node("techx")

scan_sub = rospy.Subscriber('/scan', LaserScan, disparity_extender_callback)

drive_pub = rospy.Publisher('/tianracer/ackermann_cmd', AckermannDrive, queue_size=1)

rospy.spin()
