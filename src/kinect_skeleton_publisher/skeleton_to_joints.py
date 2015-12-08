#!/usr/bin/env python
import tf
from sympy import *
from sympy.mpmath import norm
import math
import rospy
import rospkg
import json
from collections import deque
import numpy as np
from .joint_transformations import *

class SkeletonConverter:
    def __init__(self, calibrated=False, window=2):
        self.rospack = rospkg.RosPack()
        self.tf_listener = tf.TransformListener()
        self.skel_data = {}
        self.lengths = {}
        self.calibrated = calibrated
        self.calibration = self.calibration_matrices(rospy.get_param('/kinect/human_calibration')) if self.calibrated else {}
        # list of all human frames
        self.frame_list = ['head', 'neck', 'torso', 'waist',
                            'left_shoulder', 'left_elbow', 'left_wrist', 'left_hand',
                            'right_shoulder', 'right_elbow', 'right_wrist', 'right_hand',
                            'left_hip', 'left_knee', 'left_ankle', 'left_foot',
                            'right_hip', 'right_knee', 'right_ankle', 'right_foot']
        self.tf_prefix = '/kinect'
        self.joint_states = {}
        self.kinect_tf = [[0,0,0],[0,0,0,1]]
        self.init_joints(window)

    def init_joints(self, window):
        # create the list of all joints byt appending first spherical joints
        joint_list = []
        for i in range(3):
            joint_list.append('spine_'+str(i))
            joint_list.append('neck_'+str(i))
            joint_list.append('right_shoulder_'+str(i))
            joint_list.append('left_shoulder_'+str(i))
            joint_list.append('right_hip_'+str(i))
            joint_list.append('left_hip_'+str(i))
        # then append universal joints
        for i in range(2):
            joint_list.append('right_elbow_'+str(i))
            joint_list.append('left_elbow_'+str(i))
            joint_list.append('right_wrist_'+str(i))
            joint_list.append('left_wrist_'+str(i))
            joint_list.append('right_ankle_'+str(i))
            joint_list.append('left_ankle_'+str(i))
        # finish by the knee joint
        joint_list.append('right_knee')
        joint_list.append('left_knee')
        # create the dictionnary with a deque of fixe size for each joints
        for joint in joint_list:
            self.joint_states[joint] = deque(maxlen=window)

    def calibrate(self):
        if self.update_skeleton():
            self.generate_model()
            # generate self.calibration and store on the parameter server
            with open(self.rospack.get_path("kinect_skeleton_publisher")+"/config/calibration_matrices.json") as data_file:
                data = json.load(data_file)
            rospy.set_param('/kinect/human_calibration', data)
            return True
        else:
            return False

    def generate_model(self):
        self.lengths = {'upper_arm_length': float(norm(self.skel_data['right_elbow'][:-1,-1])),
                        'forearm_length': float(norm(self.skel_data['right_wrist'][:-1,-1])),
                        'torso_length': float(norm(self.skel_data['neck'][:-1,-1])),
                        'waist_length': float(norm(self.skel_data['torso'][:-1,-1])),
                        'neck_length': float(norm(self.skel_data['head'][:-1,-1])),
                        'thigh_length': float(norm(self.skel_data['right_knee'][:-1,-1])),
                        'shin_length': float(norm(self.skel_data['right_ankle'][:-1,-1])),
                        'shoulder_offset_width': float(abs(self.skel_data['right_shoulder'][0,-1])),
                        'shoulder_offset_height': float(abs(self.skel_data['right_shoulder'][1,-1])),
                        'hip_offset_width': float(abs(self.skel_data['right_hip'][0,-1])),
                        'hip_offset_height': float(-abs(self.skel_data['right_hip'][1,-1])),
                        'hand_length': 0.1,   # TODO
                        'foot_length': 0.1,   # TODO
                        'waist_radius': 0.12, # TODO
                        'torso_radius': 0.15, # TODO
                        'head_radius': 0.1 }  # TODO
        # set the lengths on the parameter server
        rospy.set_param('/kinect/human_lengths', self.lengths)

    def calibration_matrices(self, d):
        mat_dict = {}
        for namespace, dico in d.iteritems():
            mat_dict[namespace] = {}
            for key, value in dico.iteritems():
                mat = eval(str(value))
                mat_dict[namespace][key] = mat
                mat_dict[namespace][key+'/inv'] = inverse(mat)
        return mat_dict

    def average_joints(self):
        avg_joints = {}
        for key, value in self.joint_states.iteritems():
            avg_joints[key] = np.mean(value)
        return avg_joints

    def convert_to_joints(self):
        def convert_body():
            # get the body frames
            spine = self.skel_data['torso']
            neck = self.skel_data['neck']
            # get joints from hip to spine
            self.joint_states['spine_0'].append(math.atan2(-spine[1,2],spine[2,2]))
            self.joint_states['spine_1'].append(math.asin(spine[0,2]))
            self.joint_states['spine_2'].append(math.atan2(-spine[0,1],spine[0,0]))
            # get joints from neck to head
            self.joint_states['neck_0'].append(math.atan2(-neck[1,2],neck[2,2]))
            self.joint_states['neck_1'].append(math.asin(neck[0,2]))
            self.joint_states['neck_2'].append(math.atan2(-neck[0,1],neck[0,0]))

        def convert_arm(side):
            shoulder = self.skel_data[side+'_shoulder']
            elbow = self.skel_data[side+'_elbow']
            wrist = self.skel_data[side+'_wrist']
            # sign differences between sides
            if side == 'right':
                # get shoulder joints
                self.joint_states[side+'_shoulder_0'].append(math.atan2(shoulder[0,0],-shoulder[1,0]))
            else:
                # get shoulder joints
                self.joint_states[side+'_shoulder_0'].append(math.atan2(-shoulder[0,0],shoulder[2,2]))
            self.joint_states[side+'_shoulder_1'].append(math.asin(-shoulder[2,0]))
            self.joint_states[side+'_shoulder_2'].append(math.atan2(shoulder[2,1],shoulder[2,2]))
            # get elbow joints
            self.joint_states[side+'_elbow_0'].append(math.atan2(elbow[1,0],elbow[0,0]))
            self.joint_states[side+'_elbow_1'].append(math.atan2(elbow[2,1],elbow[2,2]))
            # get wrist joints
            self.joint_states[side+'_wrist_0'].append(math.atan2(-wrist[2,0],wrist[2,2]))
            self.joint_states[side+'_wrist_1'].append(math.atan2(-wrist[0,1],wrist[1,1]))

        def convert_leg(side):
            hip = self.skel_data[side+'_hip']
            knee = self.skel_data[side+'_knee']
            ankle = self.skel_data[side+'_ankle']
            # get hip joints
            self.joint_states[side+'_hip_0'].append(math.atan2(hip[1,0],-hip[2,0]))
            self.joint_states[side+'_hip_1'].append(math.asin(-hip[0,0]))
            self.joint_states[side+'_hip_2'].append(math.atan2(hip[0,1],hip[0,2]))
            # get knee joint
            self.joint_states[side+'_knee'].append(math.atan2(-knee[0,1],knee[0,0]))
            # get ankle joints
            self.joint_states[side+'_ankle_0'].append(math.atan2(ankle[2,1],-ankle[0,1]))
            self.joint_states[side+'_ankle_1'].append(math.atan2(-ankle[1,0],ankle[1,2]))

        convert_body()
        convert_arm('right')
        convert_arm('left')
        convert_leg('right')
        convert_leg('left')
        return self.average_joints()

    def is_skeleton_visible(self):
        visible = True
        for frame in self.frame_list:
            visible = visible and self.tf_listener.frameExists(self.tf_prefix+frame)
        return visible

    def update_kinect_transformation(self, tilt):
        if self.tf_listener.canTransform('kinect_frame', self.tf_prefix+'/human/waist', rospy.Time(0)):
            time = self.tf_listener.getLatestCommonTime('kinect_frame', self.tf_prefix+'/human/waist')
            if rospy.Time.now() - time < rospy.Duration(0.5):
                kinect_frame = tf_to_matrix(self.tf_listener.lookupTransform('kinect_frame', self.tf_prefix+'/human/waist', time))
                # multiply with the calibration
                kinect_frame = kinect_frame*self.calibration['/kinect']['waist']
                # if we need to counteract the hip tilt, multiply by other transform
                if tilt:
                    kinect_frame = kinect_frame*self.calibration['/kinect']['tilt']
                # convert to tf
                pose = sympy_to_numpy(kinect_frame)
                self.kinect_tf[0] = pose[:-1,-1].tolist()
                self.kinect_tf[1] = tf.transformations.quaternion_from_matrix(pose)
                return True
        return False

    def update_skeleton(self):    
        def update_frame(base, target, base_prefix=None, target_prefix=None):
            visible = False
            if base_prefix is None:
                base_prefix = [self.tf_prefix]
            if target_prefix is None:
                target_prefix = [self.tf_prefix]
            # loop through all the prefixes
            for b_pref in base_prefix:
                for t_pref in target_prefix:
                    # check visibility of frame
                    if self.tf_listener.canTransform(b_pref+'/human/'+base, t_pref+'/human/'+target, rospy.Time(0)):
                        time = self.tf_listener.getLatestCommonTime(b_pref+'/human/'+base, t_pref+'/human/'+target)
                        if rospy.Time.now() - time < rospy.Duration(0.5):
                            self.skel_data[target] = tf_to_matrix(self.tf_listener.lookupTransform(b_pref+'/human/'+base, t_pref+'/human/'+target, time))
                            # multiply each transformation by the calibration matrix
                            if self.calibrated:
                                self.skel_data[target] = self.calibration[b_pref][base+'/inv']*self.skel_data[target]*self.calibration[t_pref][target]
                            visible = True
                            break
                if visible:
                    break
            if not visible:
                self.visible = False
        def update_body():
            update_frame('waist', 'torso')
            update_frame('torso', 'neck')
            update_frame('neck', 'head')
        def update_arm(side):
            update_frame('torso', side+'_shoulder')
            update_frame(side+'_shoulder', side+'_elbow', target_prefix=['/optitrack', '/kinect'])
            update_frame(side+'_elbow', side+'_wrist')
            update_frame(side+'_wrist', side+'_hand')
        def update_leg(side):
            update_frame('waist', side+'_hip')
            update_frame(side+'_hip', side+'_knee')
            update_frame(side+'_knee', side+'_ankle')
            update_frame(side+'_ankle', side+'_foot')
        # set the visibility boolean
        self.visible = True
        # get the transformation for the body
        update_body()
        # get the transformation for both arms
        update_arm('right')
        update_arm('left')
        # get the transformation for both legs
        update_leg('right')
        update_leg('left')
        return self.visible


