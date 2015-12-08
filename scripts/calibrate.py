#!/usr/bin/env python
from kinect_skeleton_publisher.skeleton_to_joints import SkeletonConverter
from tf import LookupException
import rospy
import rospkg
import xacro
import os
import sys

class Calibrator():
    def __init__(self, use_timer=True):
        self.use_timer = use_timer
        self.rospack = rospkg.RosPack()
        self.document = None
        self.skel = SkeletonConverter(False)

    def request_for_t_position(self):
        while not rospy.is_shutdown():
            if self.use_timer:
                rospy.sleep(5)
                os.system('beep')
            else:
                raw_input("User in T position, press <Enter> when ready...")
            if self.skel.calibrate():
                rospy.loginfo("T pose recorded successfully")
                break
            else:
                rospy.logerr("Cannot calibrate, skeleton not visible")

    def create_urdf(self, output_filename=None, output_param='/human_description'):
        str_lengths = { k:str(v) for k, v in self.skel.lengths.iteritems() }
        xacro.set_substitution_args_context(str_lengths)

        with open(self.rospack.get_path("kinect_skeleton_publisher")+"/urdf/human_skeleton.urdf.xacro") as f:
            self.document = xacro.parse(f)

        xacro.process_includes(self.document, self.rospack.get_path("kinect_skeleton_publisher")+"/urdf/")
        xacro.eval_self_contained(self.document)

        if output_param is not None:
            rospy.set_param(output_param, self.document.toprettyxml(indent='  '))

        if output_filename is not None:
            with open(output_filename, 'w') as f:
                f.write(self.document.toprettyxml(indent='  '))

if __name__=='__main__':
    rospy.init_node('kinect_skeleton_calibrator')
    calibrator = Calibrator(sys.argv[1]=='True')
    calibrator.request_for_t_position()
    output_urdf = calibrator.rospack.get_path("kinect_skeleton_publisher")+"/urdf/people/"+rospy.get_param('/kinect/person_name')+'.urdf'
    calibrator.create_urdf(output_urdf)
