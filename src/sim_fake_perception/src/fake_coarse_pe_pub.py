#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from vader_msgs.msg import Pepper, Fruit, Peduncle, SimulationPepperSequence, SimulationPopPepper
import random
from tf.transformations import quaternion_from_euler
import numpy as np

pepper_sequence = None

def create_pepper(fruit_pose, fruit_shape, peduncle_shape):
    pepper = Pepper()
    pepper.header.stamp = rospy.Time.now()
    pepper.header.frame_id = "link_base"

    # Create the fruit
    fruit = Fruit()
    fruit.pose = fruit_pose
    fruit.shape = fruit_shape
    pepper.fruit_data = fruit

    # print("Fruit:", fruit)

    # Create the peduncle
    peduncle = Peduncle()
    peduncle.pose = fruit_pose
    peduncle.shape = peduncle_shape
    peduncle.pose.position.z = fruit_pose.position.z + fruit.shape.dimensions[0] / 2 + peduncle.shape.dimensions[0] / 2
    pepper.peduncle_data = peduncle

    return pepper

def get_gaussian_xyz_noise(xyz_noise):
    return np.random.normal(0, xyz_noise, 3)

def _get_sim_pepper_sequence(data):
    global pepper_sequence
    pepper_sequence = data.pepper_poses
    # print("peppers:", pepper_sequence)

def _pop_current_pepper(data):
    global pepper_sequence
    if len(pepper_sequence) > 0:
        pepper_sequence.pop(0)
        # print("popped")

def publisher():
    rospy.init_node('vader_pepper_publisher', anonymous=True)
    # pepper_pose = rospy.get_param('~sim_pepper_pose', "0.0 0.0 0.0 0.0 0.0 0.0")
    # pepper_pose = list(map(float, pepper_pose.split(" ")))
    pepper_seq_sub = rospy.Subscriber("/pepper_sequence", SimulationPepperSequence, _get_sim_pepper_sequence)
    pepper_pop_sub = rospy.Subscriber("/pepper_pop", SimulationPopPepper, _pop_current_pepper)
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
    pub_all_coarse_topic = rospy.Publisher("/fruit_coarse_pose_remaining", SimulationPepperSequence, queue_size=10)

    global pepper_sequence
    while pepper_sequence is None:
        rospy.sleep(0.1)

    pub = rospy.Publisher(pub_topic, Pepper, queue_size=10)
    rate = rospy.Rate(pub_hz) 

    rospy.sleep(1) # Wait for the publisher to be registered

    while not rospy.is_shutdown():
        # Add noise to the gt_pose
        _noise_xyz = get_gaussian_xyz_noise(xyz_noise)
        _pepper_pose = Pose()
        if pepper_sequence is None or len(pepper_sequence) == 0:
            rospy.loginfo("No pepper sequence available, exiting")
            break
        _pepper_pose.position.x = pepper_sequence[0].position.x + _noise_xyz[0]
        _pepper_pose.position.y = pepper_sequence[0].position.y + _noise_xyz[1]
        _pepper_pose.position.z = pepper_sequence[0].position.z + _noise_xyz[2]
        _pepper_pose.orientation.x = 0.
        _pepper_pose.orientation.y = 0.
        _pepper_pose.orientation.z = 0.
        _pepper_pose.orientation.w = 1.

        # Create pepper
        pepper = create_pepper(_pepper_pose, fruit_shape, peduncle_shape)

        # rospy.loginfo(f"Publishing Coarse Pose Pepper: {pepper}")
        pub.publish(pepper)
        
        remaining_peppers = []
        # Publish remaining peppers in queue
        for i in range(1, len(pepper_sequence)):
            #_noise_xyz = get_gaussian_xyz_noise(xyz_noise)
            _pepper_pose = Pose()
            _pepper_pose.position.x = pepper_sequence[i].position.x #+ _noise_xyz[0] # due to thing in HRI that flips these polarities. Need to fix after 
            _pepper_pose.position.y = pepper_sequence[i].position.y #+ _noise_xyz[1]
            _pepper_pose.position.z = pepper_sequence[i].position.z #+ _noise_xyz[2]
            _pepper_pose.orientation.x = 0.
            _pepper_pose.orientation.y = 0.
            _pepper_pose.orientation.z = 0.
            _pepper_pose.orientation.w = 1.

            # rospy.loginfo(f"Publishing Coarse Pose Pepper: {pepper}")
            remaining_peppers.append(_pepper_pose)
        # print("Remaining Peppers:", len(remaining_peppers))
        pub_all_coarse_topic.publish(remaining_peppers)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass