#!/usr/bin/env python
import vrpn
import rospy
import tf
from threading import Lock
import numpy as np
import zmq
import json


class KinectSkeletonPublisher():
    def __init__(self, ip, port, num_skeletons=1, rate=50, use_v2=False):
        self.use_v2 = use_v2
        self.num_skeletons = num_skeletons
        self.rate = rospy.Rate(rate)
        self.tfb = tf.TransformBroadcaster()
        self.trackers = []
        # namespace for tf publication
        self.tf_prefix = '/kinect/human/'
        self.joint_names = ['head', 'neck', 'spine', 'base',
                            'left_shoulder', 'left_elbow', 'left_wrist', 'left_hand',
                            'right_shoulder', 'right_elbow', 'right_wrist', 'right_hand',
                            'left_hip', 'left_knee', 'left_ankle', 'left_foot',
                            'right_hip', 'right_knee', 'right_ankle', 'right_foot']
        # initalize mutex lock
        self.lock = Lock()
        if self.use_v2:
            print 'kinect version 2'
            # add frames to publisher
            self.joint_names.append('shoulder_center')
            # create the dict of equivalence
            self.equiv_dict = {}
            self.equiv_dict['head'] = 'Head'
            self.equiv_dict['neck'] = 'Neck'
            self.equiv_dict['shoulder_center'] = 'SpineShoulder'
            self.equiv_dict['spine'] = 'SpineMid'
            self.equiv_dict['base'] = 'SpineBase'
            sides = ['Right', 'Left']
            for s in sides:
                self.equiv_dict[s.lower() + '_shoulder'] = 'Shoulder' + s
                self.equiv_dict[s.lower() + '_elbow'] = 'Elbow' + s
                self.equiv_dict[s.lower() + '_wrist'] = 'Wrist' + s
                self.equiv_dict[s.lower() + '_hand'] = 'Hand' + s
                self.equiv_dict[s.lower() + '_hip'] = 'Hip' + s
                self.equiv_dict[s.lower() + '_knee'] = 'Knee' + s
                self.equiv_dict[s.lower() + '_ankle'] = 'Ankle' + s
                self.equiv_dict[s.lower() + '_foot'] = 'Foot' + s
            # connect to ZMQ
            context = zmq.Context()
            self.socket = context.socket(zmq.SUB)
            self.socket.setsockopt(zmq.SUBSCRIBE, "skeleton")
            self.socket.connect("tcp://" + ip + ":" + port)
        else:
            # connect to VRPN
            for skeleton in range(self.num_skeletons):
                name = 'Tracker' + str(skeleton)
                t = vrpn.receiver.Tracker('{}@{}:{}'.format(name, ip, port))
                t.register_change_handler(name, self.handler_v1, 'position')
                self.trackers.append(t)

        # create the dictionnary of frames
        self.frames = {}
        for joint in self.joint_names:
            self.frames[joint] = {}
            self.frames[joint]['empty'] = True
            self.frames[joint]['position'] = [0, 0, 0]
            self.frames[joint]['quaternion'] = [0, 0, 0, 1]

    def handler_v1(self, name, data):
        sensor = data['sensor']
        with self.lock:
            self.frames[self.joint_names[sensor]]['empty'] = False
            self.frames[self.joint_names[sensor]]['position'] = data['position']
            self.frames[self.joint_names[sensor]]['quaternion'] = data['quaternion']

    def handler_v2(self):
        skeletons = self.socket.recv()
        skeletons = json.loads(skeletons.split(' ')[1])
        for key, skeleton in skeletons.iteritems():
            for joint in self.joint_names:
                with self.lock:
                    self.frames[joint]['empty'] = False
                    value = skeleton[self.equiv_dict[joint]]
                    pos = [value['Position']['X'], value['Position']['Y'], value['Position']['Z']]
                    self.frames[joint]['position'] = pos
                    rot = [value['Orientation']['X'], value['Orientation']['Y'], value['Orientation']['Z'], value['Orientation']['W']]
                    self.frames[joint]['quaternion'] = rot

    def send_tf(self, joint, pose):
        pos = pose['position']
        rot = pose['quaternion']
        self.tfb.sendTransform(pos, rot, rospy.Time.now(), self.tf_prefix + joint, "kinect_frame")

    def send_shoulder_center(self):
        poseR = self.frames['right_shoulder']
        poseL = self.frames['left_shoulder']
        pos = ((np.array(poseR['position']) + np.array(poseL['position'])) / 2).tolist()
        rot = poseR['quaternion']
        self.tfb.sendTransform(pos, rot, rospy.Time.now(), self.tf_prefix + 'shoulder_center', "kinect_frame")

    def run(self):
        while not rospy.is_shutdown():
            if self.use_v2:
                self.handler_v2()
            else:
                for t in self.trackers:
                    t.mainloop()
            with self.lock:
                if not self.use_v2:
                    # publish shoulder_center frame
                    empty = (self.frames['right_shoulder']['empty'] or self.frames['left_shoulder']['empty'])
                    if not empty:
                        self.send_shoulder_center()
                for joint, pose in self.frames.iteritems():
                    if not pose['empty']:
                        self.send_tf(joint, pose)
                        pose['empty'] = True
                self.joints = {}
            if not self.use_v2:
                self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('kinect_skeleton_publisher')
    ip = rospy.get_param('/kinect/vrpn_ip')
    port = rospy.get_param('/kinect/vrpn_port')
    num_skeletons = rospy.get_param('/kinect/num_skeletons')
    use_v2 = rospy.get_param('/kinect/use_v2')
    KinectSkeletonPublisher(ip, port, int(num_skeletons), use_v2=use_v2).run()
