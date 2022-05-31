#!/usr/bin/env python3

import numpy as np
import time

import rospy

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from lio_sam.msg import Float64MultiArrayStamped

import transforms3d

def pose_to_twist(pose_msg, A_prev, dt):
    # https://matthew-brett.github.io/transforms3d/reference/transforms3d.affines.html

    p = pose_msg.position
    q = pose_msg.orientation

    T = [p.x, p.y, p.z]
    R = transforms3d.quaternions.quat2mat([q.w, q.x, q.y, q.z])

    A = transforms3d.affines.compose(T, R)

    A_rel = A @ np.linalg.inv(A_prev)

    T, R = transforms3d.affines.decompose44(A_rel)

    return T, R, A

def pose_msg_to_np(pose_msg):
    """
    Convert Pose msg to numpy array 

    Args:
      state_np: state
    """

    pose_np = np.zeros(6)
    pose_np[0] = pose_msg.position.x
    pose_np[1] = pose_msg.position.y
    pose_np[2] = pose_msg.position.z
    q = pose_msg.orientation
    pose_np[3], pose_np[4], pose_np[5] = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])

    return pose_np


def quaternion_msg_to_np(q_msg):

    q_np = np.array([q_msg.w, q_msg.x, q_msg.y, q_msg.z])

    return q_np

def calc_dpose_msg(pose_msg, prev_pose_msg):
    """
    Convert Pose msg to numpy array 

    Args:
      state_np: state
    """

    dpose_msg = Pose()

    dpose_msg.position.x = pose_msg.position.x - prev_pose_msg.position.x
    dpose_msg.position.y = pose_msg.position.y - prev_pose_msg.position.y

    q_np = quaternion_msg_to_np(pose_msg.orientation)
    prev_q_np = quaternion_msg_to_np(prev_pose_msg.orientation)

    dq_np = transforms3d.quaternions.qmult(transforms3d.quaternions.qinverse(prev_q_np), q_np)

    dpose_msg.orientation.w = dq_np[0]
    dpose_msg.orientation.x = dq_np[1]
    dpose_msg.orientation.y = dq_np[2]
    dpose_msg.orientation.z = dq_np[3]

    return dpose_msg

class OdomCompare():

    # https://gist.github.com/gajena/ed61f7eea7c6a967cc9a4171a07fc13f

    def __init__(self):
        rospy.Subscriber("/lio_sam/lidar_pose3_factor", PoseStamped, self.lio_pose_cb)
        rospy.Subscriber("/odometry/gt/global", Odometry, self.odom_gt_cb)

        self.gt_dpose_pub = rospy.Publisher('/lio_sam/gt_dpose', PoseStamped, queue_size=10)
        self.lio_dpose_pub = rospy.Publisher('/lio_sam/lio_dpose', PoseStamped, queue_size=10)

        self.dpose_error_pub = rospy.Publisher('/lio_sam/debug/dpose_error', Float64MultiArrayStamped, queue_size=10)        

        # self.A_gt_prev = np.zeros((4,4))
        # self.A_lio_prev = np.zeros((4,4))

        self.pose_gt_prev = None #Pose()

        self.prev_pose_lio_np = None

    def odom_gt_cb(self, data):

        self.pose_gt_msg = data.pose

    def lio_pose_cb(self, data):

        pose_lio_msg = data.pose

        # pose_lio_np = pose_msg_to_np(pose_lio_msg)       

        dt = 0.0
        # T_gt, R_gt, self.A_gt_prev = pose_to_twist(self.pose_gt_msg, self.A_gt_prev, dt)

        if self.pose_gt_prev is not None:

            gt_dpose_msg = PoseStamped()

            # gt_dpose_msg.pose.position.x = T_gt[0]
            # gt_dpose_msg.pose.position.y = T_gt[1]
            # gt_dpose_msg.pose.position.z = T_gt[2]

            gt_dpose_msg.pose = calc_dpose_msg(self.pose_gt_msg.pose, self.pose_gt_prev)

            # gt_dpose_msg.pose.position.x = self.pose_gt_msg.pose.position.x - self.pose_gt_prev.position.x
            # gt_dpose_msg.pose.position.y = self.pose_gt_msg.pose.position.y - self.pose_gt_prev.position.y

            self.gt_dpose_pub.publish(gt_dpose_msg)

           
            # self.lio_dpose_pub.publish(lio_dpose_msg)

            gt_q_np = quaternion_msg_to_np(gt_dpose_msg.pose.orientation)
            lio_q_np = quaternion_msg_to_np(pose_lio_msg.orientation)
            dq_np = transforms3d.quaternions.qmult(transforms3d.quaternions.qinverse(lio_q_np), gt_q_np)
            rpy = transforms3d.euler.quat2euler(dq_np)

            dpose_error_msg = Float64MultiArrayStamped()
            dpose_error_msg.multi_array.data = rpy
            self.dpose_error_pub.publish(dpose_error_msg)

        self.pose_gt_prev = self.pose_gt_msg.pose

        # self.prev_pose_lio_np = pose_lio_np
















if __name__=='__main__':

    rospy.init_node('odom_compare', anonymous=True, disable_signals=True)
    oc = OdomCompare()
    rospy.spin()
