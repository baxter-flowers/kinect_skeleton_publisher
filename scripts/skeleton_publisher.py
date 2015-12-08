#!/usr/bin/env python
import vrpn, time, rospy, tf
from kinect_skeleton_publisher.joint_transformations import *
from sympy import pi
from threading import Lock

class KinectSkeletonPublisher():
    def __init__(self, ip, port, num_skeletons=1, rate=50):
        self.ip = ip
        self.port = port
        self.num_skeletons = num_skeletons
        self.rate = rospy.Rate(rate)
        self.tfb = tf.TransformBroadcaster()
        self.trackers = []
        # transformation matrix from kinect system to standardized system
        self.system_transform = rotation_y(pi/2)*rotation_z(pi/2)
        # namespace for tf publication
        self.tf_prefix = '/kinect/human/'
        self.joint_names = ['head', 'neck', 'torso', 'waist',
                            'left_shoulder', 'left_elbow', 'left_wrist', 'left_hand',
                            'right_shoulder', 'right_elbow', 'right_wrist', 'right_hand',
                            'left_hip', 'left_knee', 'left_ankle', 'left_foot',
                            'right_hip', 'right_knee', 'right_ankle', 'right_foot']
        self.frames = {}
        # create the dictionnary of frames
        for joint in self.joint_names:
            self.frames[joint] = {}
            self.frames[joint]['empty'] = True
            self.frames[joint]['position'] = [0,0,0]
            self.frames[joint]['quaternion'] = [0,0,0,1]
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
        # convert tf to transformation matrix
        T = tf_to_matrix((pose['position'], pose['quaternion']))
        # apply the system transformation
        pose = self.system_transform*T
        # convert it back to tf
        pose = sympy_to_numpy(pose)
        quaternion = tf.transformations.quaternion_from_matrix(pose)
        position = pose[:-1,-1].tolist()
        # send tf
        self.tfb.sendTransform(position, quaternion, rospy.Time.now(), self.tf_prefix + joint, "kinect_frame")


    def run(self):
        while not rospy.is_shutdown():
            for t in self.trackers:
                t.mainloop()
            with self.lock:
                for joint, pose in self.frames.iteritems():
                    if not pose['empty']:
                        self.send_tf(joint, pose)
                        pose['empty'] = True
                self.joints = {}
            self.rate.sleep()


if __name__=='__main__':
    rospy.init_node('kinect_skeleton_publisher')
    ip = rospy.get_param('/kinect/vrpn_ip')
    port = rospy.get_param('/kinect/vrpn_port')
    num_skeletons = rospy.get_param('/kinect/num_skeletons')
    KinectSkeletonPublisher(ip, port, int(num_skeletons)).run()
