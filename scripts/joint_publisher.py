#!/usr/bin/env python
from kinect_skeleton_publisher.skeleton_to_joints import SkeletonConverter
from sensor_msgs.msg import JointState
import tf
import rospy
import os

class JointPublisher:
    def __init__(self, rate=10):
        self.rate = rospy.Rate(rate)
        self.calibrated = False
        self.joint_states_publisher = rospy.Publisher('/human/joint_states', JointState, queue_size=1)
        try:
            self.skeleton = SkeletonConverter(True)
        except KeyError:
            rospy.logerr("Please calibrate before running the joint publisher")
        else:
            self.calibrated = True
            rospy.loginfo("Skeleton is calibrated!")

    def run(self):
        if self.calibrated:
            while not rospy.is_shutdown():
                if self.skeleton.update_skeleton():
                    # convert skeleton to joints
                    joint_states = self.skeleton.convert_to_joints()
                    # create joint state msg
                    joint_states_msg = JointState()
                    # update header with time and frame id
                    joint_states_msg.header.stamp = rospy.Time.now()
                    joint_states_msg.header.frame_id = '/human/hip'
                    # fill message
                    joint_states_msg.name = joint_states.keys()
                    joint_states_msg.position = joint_states.values()
                    # publish joint state
                    self.joint_states_publisher.publish(joint_states_msg)
                else:
                    rospy.logwarn('skeleton not visible')
                    rospy.sleep(1)
                self.rate.sleep()

if __name__=='__main__':
    rospy.init_node('kinect_skeleton_publisher')
    JointPublisher(10).run()

