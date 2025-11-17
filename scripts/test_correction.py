#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Simple test script to verify AR marker laser correction
"""

import rospy
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
import math

class CorrectionVerifier:
    def __init__(self):
        rospy.init_node('correction_verifier', anonymous=True)
        
        self.original_markers = None
        self.corrected_markers = None
        
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.original_callback)
        rospy.Subscriber('/corrected_ar_markers', AlvarMarkers, self.corrected_callback)
        
        rospy.loginfo("Correction verifier started")
        
    def original_callback(self, msg):
        self.original_markers = msg
        
    def corrected_callback(self, msg):
        self.corrected_markers = msg
        self.compare_markers()
        
    def compare_markers(self):
        if self.original_markers is None or self.corrected_markers is None:
            return
            
        if len(self.original_markers.markers) == 0 or len(self.corrected_markers.markers) == 0:
            return
            
        for orig, corr in zip(self.original_markers.markers, self.corrected_markers.markers):
            if orig.id != corr.id:
                continue
                
            # Extract yaw from quaternions
            orig_quat = (orig.pose.pose.orientation.x, orig.pose.pose.orientation.y,
                        orig.pose.pose.orientation.z, orig.pose.pose.orientation.w)
            corr_quat = (corr.pose.pose.orientation.x, corr.pose.pose.orientation.y,
                        corr.pose.pose.orientation.z, corr.pose.pose.orientation.w)
            
            orig_euler = tf.transformations.euler_from_quaternion(orig_quat)
            corr_euler = tf.transformations.euler_from_quaternion(corr_quat)
            
            orig_yaw = orig_euler[2] * 180.0 / math.pi
            corr_yaw = corr_euler[2] * 180.0 / math.pi
            
            diff = corr_yaw - orig_yaw
            while diff > 180:
                diff -= 360
            while diff < -180:
                diff += 360
                
            rospy.loginfo("Marker ID %d: Original yaw: %.2f deg, Corrected yaw: %.2f deg, Difference: %.2f deg" 
                         % (orig.id, orig_yaw, corr_yaw, diff))
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        verifier = CorrectionVerifier()
        verifier.run()
    except rospy.ROSInterruptException:
        pass
