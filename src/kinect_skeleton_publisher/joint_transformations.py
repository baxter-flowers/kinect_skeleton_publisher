from sympy import *
import numpy as np
import sys
import tf


def skew(u):
    u_skew = Matrix([[0,-u[2],u[1]],[u[2],0,-u[0]],[-u[1],u[0],0]])
    return u_skew


def rotation_x(theta):
    Rx = Matrix([[1,0,0,0],[0,cos(theta),-sin(theta),0],[0,sin(theta),cos(theta),0],[0,0,0,1]])
    return Rx

def rotation_y(theta):
    Ry = Matrix([[cos(theta),0,sin(theta),0],[0,1,0,0],[-sin(theta),0,cos(theta),0],[0,0,0,1]])
    return Ry

def rotation_z(theta):
    Rz = Matrix([[cos(theta),-sin(theta),0,0],[sin(theta),cos(theta),0,0],[0,0,1,0],[0,0,0,1]])
    return Rz

def rotation(u,theta):
    vect_u = Matrix(u)
    rot = vect_u*vect_u.T*(1-cos(theta)) + eye(3)*cos(theta) + skew(vect_u)*sin(theta)
    rot = rot.col_insert(3,Matrix([0,0,0]))
    rot = rot.row_insert(3,Matrix([[0,0,0,1]]))
    return rot

def translation(vect):
    T = eye(4)
    T[:-1,-1] = vect
    return T

def inversion_x():
    T = eye(4)
    T[0,0] = -1
    return T
    
def inversion_y():
    T = eye(4)
    T[1,1] = -1
    return T

def inversion_z():
    T = eye(4)
    T[2,2] = -1
    return T

def inverse(transform):
    inv_transform = eye(4)
    rot = transform[:-1,:-1].T
    inv_transform[:-1,:-1] = rot
    inv_transform[:-1,-1] = -rot*transform[:-1,-1]
    return inv_transform

def spherical_joint(t1, t2, t3, order='zyx'):
    try:
        M1 = eval('rotation_'+order[0]+'(t1)')
        M2 = eval('rotation_'+order[1]+'(t2)')
        M3 = eval('rotation_'+order[2]+'(t3)')
    except:
        print 'Order non recognize. Please enter a combination of xyz'
        sys.exit()
    return M1*M2*M3

def quaternion_to_matrix(q):
    rot = zeros(4,4)
    rot[0,0] = 1 - 2*q[1]*q[1] - 2*q[2]*q[2]
    rot[0,1] = 2*q[0]*q[1] - 2*q[2]*q[3]
    rot[0,2] = 2*q[0]*q[2] + 2*q[1]*q[3]

    rot[1,0] = 2*q[0]*q[1] + 2*q[2]*q[3]
    rot[1,1] = 1 - 2*q[0]*q[0] - 2*q[2]*q[2]
    rot[1,2] = 2*q[1]*q[2] - 2*q[0]*q[3]

    rot[2,0] = 2*q[0]*q[2] - 2*q[1]*q[3]
    rot[2,1] = 2*q[1]*q[2] + 2*q[0]*q[3]
    rot[2,2] = 1 - 2*q[0]*q[0] - 2*q[1]*q[1]

    rot[3,3] = 1
    return rot

def tf_to_matrix(tf_list):
    trans = translation(tf_list[0])
    rot = Matrix(tf.transformations.quaternion_matrix(tf_list[1]))
    return trans*rot

def sympy_to_numpy(m):
    return np.array(np.array(m), np.float)

# convert a dictionnary of list matrices to sympy matrices
def to_matrices(d):
    mat_dict = {}
    for key, value in d.iteritems():
        mat_dict[key] = Matrix(value)
    return mat_dict

# convert a dictionnary of sympy matrices to list matrices
def to_lists(d):
    list_dict = {}
    for key, value in d.iteritems():
        list_dict[key] = sympy_to_numpy(value).tolist()
    return list_dict