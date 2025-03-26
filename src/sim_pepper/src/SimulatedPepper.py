#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose


class SimulatedPepper:
    def __init__(self, model_name='pepper', detach_topic='/detach'):
        rospy.init_node('SimPepper', anonymous=True)
        self.rate = rospy.Rate(60)  # 60Hz
        self.model_name = 'pepper'
        self.detach_requested = False  # New flag for detach status
        self.initial_pose = Pose()  # Stores initial XYZ + orientation

        # Get initial pose from Gazebo
        self._get_initial_pose()

        # Set up service proxy
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Set up detach subscriber
        rospy.Subscriber(detach_topic, Empty, self.detach_callback)

    def _get_initial_pose(self):
        """Retrieve initial pose from Gazebo when simulation starts"""
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            response = get_model_state(self.model_name, 'world')
            
            self.initial_pose = response.pose
            rospy.loginfo("\nInitial Pose Captured:")
            rospy.loginfo(f"Position (XYZ): [{self.initial_pose.position.x:.2f}, "
                        f"{self.initial_pose.position.y:.2f}, "
                        f"{self.initial_pose.position.z:.2f}]")
            rospy.loginfo(f"Orientation (Quaternion): [{self.initial_pose.orientation.x:.2f}, "
                        f"{self.initial_pose.orientation.y:.2f}, "
                        f"{self.initial_pose.orientation.z:.2f}, "
                        f"{self.initial_pose.orientation.w:.2f}]")
        except rospy.ServiceException as e:
            rospy.logerr(f"Initial pose acquisition failed: {e}")
            raise

    def detach_callback(self, msg):
        rospy.loginfo("Received detach command")
        self.detach_requested = True

    def set_pose(self):
        model_state = ModelState()
        model_state.model_name = self.model_name
        model_state.pose.position.x = self.initial_pose.position.x
        model_state.pose.position.y = self.initial_pose.position.y
        model_state.pose.position.z = self.initial_pose.position.z
        model_state.pose.orientation.x = self.initial_pose.orientation.x
        model_state.pose.orientation.y = self.initial_pose.orientation.y
        model_state.pose.orientation.z = self.initial_pose.orientation.z
        model_state.pose.orientation.w = self.initial_pose.orientation.w

        try:
            self.set_model_state_service(model_state)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def run(self):
        while not rospy.is_shutdown() and not self.detach_requested:
            self.set_pose()
            self.rate.sleep()

if __name__ == '__main__':
    pepper = SimulatedPepper()
    pepper.run()
