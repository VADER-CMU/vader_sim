#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from vader_msgs.msg import Pepper, Fruit, Peduncle, SimulationPepperSequence, SimulationPopPepper
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix


pepper_sequence = None

def create_pepper_fine(peduncle_pose, fruit_shape, peduncle_shape):
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
    fruit.pose.position.x += rotated_vector[0] 
    fruit.pose.position.y += rotated_vector[1]
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

def create_pepper_coarse(fruit_pose, fruit_shape, peduncle_shape):
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

def get_gaussian_noise(noise_amplitude):
    return np.random.normal(0, noise_amplitude, 3)

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
    xyz_noise_coarse = rospy.get_param('~xyz_noise_coarse', "0.01")
    xyz_noise_fine = rospy.get_param('~xyz_noise_fine', "0.01")
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
    pub_topic_fine = rospy.get_param('~pub_topic', "/fruit_fine_pose")
    pub_topic_coarse = rospy.get_param('~pub_topic_coarse', "/fruit_coarse_pose")
    pub_all_coarse_topic = rospy.Publisher("/fruit_coarse_pose_remaining", SimulationPepperSequence, queue_size=10)

    global pepper_sequence
    while pepper_sequence is None:
        rospy.sleep(0.1)

    pub_fine = rospy.Publisher(pub_topic_fine, Pepper, queue_size=10)
    pub_coarse = rospy.Publisher(pub_topic_coarse, Pepper, queue_size=10)
    rate = rospy.Rate(pub_hz) 

    rospy.sleep(1) # Wait for the publisher to be registered

    while not rospy.is_shutdown():
        # Add noise to the gt_pose
        _noise_xyz_fine = get_gaussian_noise(xyz_noise_fine)
        _noise_rpy = get_gaussian_noise(rpy_noise)
        if pepper_sequence is None or len(pepper_sequence) == 0:
            rospy.loginfo("No pepper sequence available, exiting")
            break
        peduncle_pose = Pose()
        peduncle_pose.position.x = pepper_sequence[0].position.x + _noise_xyz_fine[0]
        peduncle_pose.position.y = pepper_sequence[0].position.y + _noise_xyz_fine[1]
        peduncle_pose.position.z = pepper_sequence[0].position.z + _noise_xyz_fine[2]
        roll, pitch, yaw = euler_from_quaternion([
            pepper_sequence[0].orientation.x,
            pepper_sequence[0].orientation.y,
            pepper_sequence[0].orientation.z,
            pepper_sequence[0].orientation.w
        ])
        _roll = roll + _noise_rpy[0]
        _pitch = pitch + _noise_rpy[1]
        _yaw = yaw + _noise_rpy[2]

        _quaternion = quaternion_from_euler(_roll, _pitch, _yaw)
        peduncle_pose.orientation.x = _quaternion[0]
        peduncle_pose.orientation.y = _quaternion[1]
        peduncle_pose.orientation.z = _quaternion[2]
        peduncle_pose.orientation.w = _quaternion[3]
        # Create pepper
        pepper = create_pepper_fine(peduncle_pose, fruit_shape, peduncle_shape)

        # rospy.loginfo(f"Publishing Fine Pose Pepper: {pepper}")
        pub_fine.publish(pepper)
        # Add noise to the gt_pose
        _noise_xyz_coarse = get_gaussian_noise(xyz_noise_coarse)
        _pepper_pose_coarse = Pose()
        if pepper_sequence is None or len(pepper_sequence) == 0:
            rospy.loginfo("No pepper sequence available, exiting")
            break
        _pepper_pose_coarse.position.x = pepper_sequence[0].position.x + _noise_xyz_coarse[0]
        _pepper_pose_coarse.position.y = pepper_sequence[0].position.y + _noise_xyz_coarse[1]
        _pepper_pose_coarse.position.z = pepper_sequence[0].position.z + _noise_xyz_coarse[2]
        _pepper_pose_coarse.orientation.x = 0.
        _pepper_pose_coarse.orientation.y = 0.
        _pepper_pose_coarse.orientation.z = 0.
        _pepper_pose_coarse.orientation.w = 1.

        # Create pepper
        pepper = create_pepper_coarse(_pepper_pose_coarse, fruit_shape, peduncle_shape)

        # rospy.loginfo(f"Publishing Coarse Pose Pepper: {pepper}")
        pub_coarse.publish(pepper)
        
        remaining_peppers = []
        # Publish remaining peppers in queue
        for i in range(1, len(pepper_sequence)):
            #_noise_xyz = get_gaussian_xyz_noise(xyz_noise)
            _pepper_pose_coarse = Pose()
            _pepper_pose_coarse.position.x = pepper_sequence[i].position.x #+ _noise_xyz[0] # due to thing in HRI that flips these polarities. Need to fix after 
            _pepper_pose_coarse.position.y = pepper_sequence[i].position.y #+ _noise_xyz[1]
            _pepper_pose_coarse.position.z = pepper_sequence[i].position.z #+ _noise_xyz[2]
            _pepper_pose_coarse.orientation.x = 0.
            _pepper_pose_coarse.orientation.y = 0.
            _pepper_pose_coarse.orientation.z = 0.
            _pepper_pose_coarse.orientation.w = 1.

            # rospy.loginfo(f"Publishing Coarse Pose Pepper: {pepper}")
            remaining_peppers.append(_pepper_pose_coarse)
        # print("Remaining Peppers:", len(remaining_peppers))
        pub_all_coarse_topic.publish(remaining_peppers)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass