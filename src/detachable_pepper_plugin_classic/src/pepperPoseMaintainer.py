#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState



class PepperPoseMaintainer:
    def __init__(self):
        rospy.init_node('pepper_pose_maintainer', anonymous=True)
        self.rate = rospy.Rate(60)  # 60Hz
        self.model_name = 'pepper'
        self.pose_z = 1.0

        rospy.wait_for_service('/gazebo/set_model_state')
        print("Done waiting for gazebo")
        self.set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    def set_pose(self):
        model_state = ModelState()
        model_state.model_name = self.model_name
        model_state.pose.position.z = self.pose_z

        try:
            self.set_model_state_service(model_state)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def run(self):
        while not rospy.is_shutdown():
            self.set_pose()
            self.rate.sleep()

if __name__ == '__main__':
    maintainer = PepperPoseMaintainer()
    maintainer.run()