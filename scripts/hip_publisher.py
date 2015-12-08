#!/usr/bin/env python
import rospy
from kinect_skeleton_publisher.skeleton_to_joints import SkeletonConverter
import tf
import sys

class HipPublisher:
    def __init__(self, rate=10):
        self.rate = rospy.Rate(rate)
        self.calibrated = False
        # broadcaster to publish kinect transformation
        self.tfb = tf.TransformBroadcaster()
        try:
            self.skeleton = SkeletonConverter(True)
        except KeyError:
            rospy.logerr("Please calibrate before running the joint publisher")
        else:
            self.calibrated = True
            rospy.loginfo("Skeleton is calibrated!")

    def publish_kinect_transformation(self):
        # get kinect tf
        position = self.skeleton.kinect_tf[0]
        quaternion = self.skeleton.kinect_tf[1]
        # send tf
        self.tfb.sendTransform(position, quaternion, rospy.Time.now(), '/human/hip', 'kinect_frame')

    def run(self, correct_tilt=False):
        if self.calibrated:
            while not rospy.is_shutdown():
                if self.skeleton.update_kinect_transformation(correct_tilt):
                    # publish kinect transformation
                    self.publish_kinect_transformation()
                else:
                    rospy.logwarn('skeleton not visible')
                    rospy.sleep(1)
                self.rate.sleep()

if __name__=='__main__':
    rospy.init_node('hip_publisher')
    HipPublisher(10).run(sys.argv[1]=='True')