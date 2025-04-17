#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from vader_msgs.msg import Pepper, Fruit, Peduncle
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix

def create_pepper(peduncle_pose, fruit_shape, peduncle_shape):
    pepper = Pepper()
    pepper.header.stamp = rospy.Time.now()
    pepper.header.frame_id = "link_base"
    fruit_down_offset = -( fruit_shape.dimensions[0] / 2 + peduncle_shape.dimensions[0] / 2) - 0.01

    # Create the fruit
    fruit = Fruit()
    fruit.pose = Pose()
    fruit.pose.position.x = peduncle_pose.position.x
    fruit.pose.position.y = peduncle_pose.position.y
    fruit.pose.position.z = peduncle_pose.position.z
    fruit.pose.orientation.x = peduncle_pose.orientation.x
    fruit.pose.orientation.y = peduncle_pose.orientation.y
    fruit.pose.orientation.z = peduncle_pose.orientation.z
    fruit.pose.orientation.w = peduncle_pose.orientation.w
    fruit.shape = fruit_shape
    pepper.fruit_data = fruit

    # print("Fruit:", fruit)

    # Create the peduncle
    peduncle = Peduncle()
    peduncle.shape = peduncle_shape
    peduncle.pose = Pose()
    unit_vector = np.array([0, 0, fruit_down_offset])  # Example unit vector along z-axis
    quaternion = [
        fruit.pose.orientation.x,
        fruit.pose.orientation.y,
        fruit.pose.orientation.z,
        fruit.pose.orientation.w
    ]
    rotated_vector = quaternion_matrix(quaternion)[:3, :3].dot(unit_vector)

    # print(rotated_vector)
    fruit.pose.position.x += rotated_vector[1] 
    fruit.pose.position.y -= rotated_vector[0]
    fruit.pose.position.z += rotated_vector[2]

    peduncle.pose.position.x = peduncle_pose.position.x 
    peduncle.pose.position.y = peduncle_pose.position.y 
    peduncle.pose.position.z = peduncle_pose.position.z
    peduncle.pose.orientation.x = peduncle_pose.orientation.x
    peduncle.pose.orientation.y = peduncle_pose.orientation.y
    peduncle.pose.orientation.z = peduncle_pose.orientation.z
    peduncle.pose.orientation.w = peduncle_pose.orientation.w

    # print("Peduncle:", peduncle.pose.position)
    # print("Fruit:", fruit.pose.position)

    pepper.peduncle_data = peduncle
    return pepper

def get_gaussian_noise(noise_amplitude):
    return np.random.normal(0, noise_amplitude, 3)

def publisher():
    rospy.init_node('vader_pepper_publisher', anonymous=True)
    pepper_pose = rospy.get_param('~sim_pepper_pose', "0.0 0.0 0.0 0.0 0.0 0.0")
    pepper_pose = list(map(float, pepper_pose.split(" ")))
    xyz_noise = rospy.get_param('~xyz_noise', "0.01")
    rpy_noise = rospy.get_param('~rpy_noise', "0.01")
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
    pub_topic = rospy.get_param('~pub_topic', "/fruit_fine_pose")

    gt_pose = Pose()
    gt_pose.position.x = float(pepper_pose[0])
    gt_pose.position.y = float(pepper_pose[1])
    gt_pose.position.z = float(pepper_pose[2])
    # Convert RPY to quaternion
    roll, pitch, yaw = float(pepper_pose[3]), float(pepper_pose[4]), float(pepper_pose[5])

    print("Fine fruit pose estimate has ground truth =", pepper_pose)
    pub = rospy.Publisher(pub_topic, Pepper, queue_size=10)
    rate = rospy.Rate(pub_hz) 

    rospy.sleep(1) # Wait for the publisher to be registered

    while not rospy.is_shutdown():
        # Add noise to the gt_pose
        _noise_xyz = get_gaussian_noise(xyz_noise)
        _noise_rpy = get_gaussian_noise(rpy_noise)
        peduncle_pose = Pose()
        peduncle_pose.position.x = gt_pose.position.x + _noise_xyz[0]
        peduncle_pose.position.y = gt_pose.position.y + _noise_xyz[1]
        peduncle_pose.position.z = gt_pose.position.z + _noise_xyz[2]
        _roll = roll + _noise_rpy[0]
        _pitch = pitch + _noise_rpy[1]
        _yaw = yaw + _noise_rpy[2]

        _quaternion = quaternion_from_euler(_roll, _pitch, _yaw)
        peduncle_pose.orientation.x = _quaternion[0]
        peduncle_pose.orientation.y = _quaternion[1]
        peduncle_pose.orientation.z = _quaternion[2]
        peduncle_pose.orientation.w = _quaternion[3]
        # Create pepper
        pepper = create_pepper(peduncle_pose, fruit_shape, peduncle_shape)

        # rospy.loginfo(f"Publishing Fine Pose Pepper: {pepper}")
        pub.publish(pepper)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass