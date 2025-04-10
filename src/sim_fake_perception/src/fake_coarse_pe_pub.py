#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from vader_msgs.msg import Pepper, Fruit, Peduncle
import random
from tf.transformations import quaternion_from_euler
import numpy as np

def create_pepper(fruit_pose, fruit_shape, peduncle_shape):
    pepper = Pepper()
    pepper.header.stamp = rospy.Time.now()
    pepper.header.frame_id = "link_base"

    # Create the fruit
    fruit = Fruit()
    fruit.pose = fruit_pose
    fruit.shape = fruit_shape
    pepper.fruit_data = fruit

    print("Fruit:", fruit)

    # Create the peduncle
    peduncle = Peduncle()
    peduncle.pose = fruit_pose
    peduncle.shape = peduncle_shape
    peduncle.pose.position.z = fruit_pose.position.z + fruit.shape.dimensions[0] / 2 + peduncle.shape.dimensions[0] / 2
    pepper.peduncle_data = peduncle

    return pepper

def get_gaussian_xyz_noise(xyz_noise):
    return np.random.normal(0, xyz_noise, 3)

def publisher():
    rospy.init_node('vader_pepper_publisher', anonymous=True)
    pepper_pose = rospy.get_param('~sim_pepper_pose', "0.0 0.0 0.0 0.0 0.0 0.0")
    pepper_pose = list(map(float, pepper_pose.split(" ")))
    xyz_noise = rospy.get_param('~xyz_noise', "0.01")
    fruit_shape = rospy.get_param('~fruit_shape', "0.08 0.035")
    fruit_shape = list(map(float, fruit_shape.split(" ")))
    _fruit_shape = SolidPrimitive()
    _fruit_shape.type = SolidPrimitive.CYLINDER
    _fruit_shape.dimensions = fruit_shape
    fruit_shape = _fruit_shape
    peduncle_shape = rospy.get_param('~peduncle_shape', "0.02 0.002")
    peduncle_shape = list(map(float, peduncle_shape.split(" ")))
    _peduncle_shape = SolidPrimitive()
    _peduncle_shape.type = SolidPrimitive.CYLINDER
    _peduncle_shape.dimensions = peduncle_shape
    peduncle_shape = _peduncle_shape
    pub_hz = rospy.get_param('~pub_hz', "10")
    pub_topic = rospy.get_param('~pub_topic', "/fruit_coarse_pose")

    gt_pose = Pose()
    gt_pose.position.x = float(pepper_pose[0])
    gt_pose.position.y = float(pepper_pose[1])
    gt_pose.position.z = float(pepper_pose[2])
    # Convert RPY to quaternion
    roll, pitch, yaw = float(pepper_pose[3]), float(pepper_pose[4]), float(pepper_pose[5])
    quaternion = quaternion_from_euler(roll, pitch, yaw)

    # Set the orientation of gt_pose
    gt_pose.orientation.x = quaternion[0]
    gt_pose.orientation.y = quaternion[1]
    gt_pose.orientation.z = quaternion[2]
    gt_pose.orientation.w = quaternion[3]

    print("Coarse fruit pose estimate has ground truth =", gt_pose)
    pub = rospy.Publisher(pub_topic, Pepper, queue_size=10)
    rate = rospy.Rate(pub_hz) 

    rospy.sleep(1) # Wait for the publisher to be registered

    while not rospy.is_shutdown():
        # Add noise to the gt_pose
        _noise_xyz = get_gaussian_xyz_noise(xyz_noise)
        _pepper_pose = Pose()
        _pepper_pose.position.x = gt_pose.position.x + _noise_xyz[0]
        _pepper_pose.position.y = gt_pose.position.y + _noise_xyz[1]
        _pepper_pose.position.z = gt_pose.position.z + _noise_xyz[2]
        _pepper_pose.orientation = gt_pose.orientation

        # Create pepper
        pepper = create_pepper(_pepper_pose, fruit_shape, peduncle_shape)

        rospy.loginfo(f"Publishing Pepper: {pepper}")
        pub.publish(pepper)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass