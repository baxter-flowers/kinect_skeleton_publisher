#!/usr/bin/env python
import vrpn
import rospy
import tf
from threading import Lock
import numpy as np


class KinectSkeletonPublisher():
    def __init__(self, ip, port, num_skeletons=1, rate=50):
        self.ip = ip
        self.port = port
        self.num_skeletons = num_skeletons
        self.rate = rospy.Rate(rate)
        self.tfb = tf.TransformBroadcaster()
        self.trackers = []
        # namespace for tf publication
        self.tf_prefix = '/kinect/human/'
        self.joint_names = ['head', 'neck', 'spine', 'waist',
                            'left_shoulder', 'left_elbow', 'left_wrist', 'left_hand',
                            'right_shoulder', 'right_elbow', 'right_wrist', 'right_hand',
                            'left_hip', 'left_knee', 'left_ankle', 'left_foot',
                            'right_hip', 'right_knee', 'right_ankle', 'right_foot']
        self.frames = {}
        # create the dictionnary of frames
        for joint in self.joint_names:
            self.frames[joint] = {}
            self.frames[joint]['empty'] = True
            self.frames[joint]['position'] = [0, 0, 0]
            self.frames[joint]['quaternion'] = [0, 0, 0, 1]
        # initalize mutex lock
        self.lock = Lock()
        # connect to VRPN
        for skeleton in range(self.num_skeletons):
            name = 'Tracker' + str(skeleton)
            t = vrpn.receiver.Tracker('{}@{}:{}'.format(name, self.ip, self.port))
            t.register_change_handler(name, self.handler, 'position')
            self.trackers.append(t)

    def handler(self, name, data):
        sensor = data['sensor']
        with self.lock:
            self.frames[self.joint_names[sensor]]['empty'] = False
            self.frames[self.joint_names[sensor]]['position'] = data['position']
            self.frames[self.joint_names[sensor]]['quaternion'] = data['quaternion']

    def send_tf(self, joint, pose):
        pos = pose['position']
        rot = pose['quaternion']
        self.tfb.sendTransform(pos, rot, rospy.Time.now(), self.tf_prefix + joint, "kinect_frame")

    def send_base_frame(self):
        poseR = self.frames['right_hip']
        poseL = self.frames['left_hip']
        pos = ((np.array(poseR['position']) + np.array(poseL['position']))/2).tolist()
        rot = poseR['quaternion']
        self.tfb.sendTransform(pos, rot, rospy.Time.now(), self.tf_prefix + 'base', "kinect_frame")

    def send_shoulder_center(self):
        poseR = self.frames['right_shoulder']
        poseL = self.frames['left_shoulder']
        pos = ((np.array(poseR['position']) + np.array(poseL['position']))/2).tolist()
        rot = poseR['quaternion']
        self.tfb.sendTransform(pos, rot, rospy.Time.now(), self.tf_prefix + 'shoulder_center', "kinect_frame")

    def run(self):
        while not rospy.is_shutdown():
            for t in self.trackers:
                t.mainloop()
            with self.lock:
                # publish base frame
                empty = (self.frames['right_hip']['empty'] or self.frames['left_hip']['empty'])
                if not empty:
                    self.send_base_frame()
                # publish shoulder_center frame
                empty = (self.frames['right_shoulder']['empty'] or self.frames['left_shoulder']['empty'])
                if not empty:
                    self.send_shoulder_center()
                for joint, pose in self.frames.iteritems():
                    if not pose['empty']:
                        self.send_tf(joint, pose)
                        pose['empty'] = True
                self.joints = {}
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('kinect_skeleton_publisher')
    ip = rospy.get_param('/kinect/vrpn_ip')
    port = rospy.get_param('/kinect/vrpn_port')
    num_skeletons = rospy.get_param('/kinect/num_skeletons')
    KinectSkeletonPublisher(ip, port, int(num_skeletons)).run()
