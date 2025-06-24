#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This script is used for publishing the end-effector tf transforms.

Controller mapping:
    /Head_Motion (HTC vive headset) -> robot head
    /Right_Buttons trackpad -> x, y motion of the mobile base
    /Right_Buttons menu button -> home right arm
    /Right_Buttons squeeze button -> activate right arm
    /Left_Buttons trackpad left / right -> yaw rotation of the mobile base
    /Left_Buttons trackpad up / down -> lift / descend torso by 5 cm
    /Left_Buttons menu button -> home left arm
    /Left_Buttons squeeze button -> activate left arm

Author: Yichen Xie


"""

import rospy
import math
import numpy as np
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix
from geometry_msgs.msg import Pose, Twist, PoseStamped, TransformStamped
from std_msgs.msg import Bool,Float32MultiArray
from numpy import linalg as LA
import tf
import transformations
from std_msgs.msg import Float64
import sys

class TiagoArmEEChecker():
    def __init__(self,
                 controller_side='right'):
        self.controller_side = controller_side  
        rospy.init_node('check_'+controller_side+'ee_loc')
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.up_limit = 0.0
        self.down_limit = -0.4
        self.extend_limit = 0.65
        self.shrink_limit = 0.42
        self.arm_extend_pub = rospy.Publisher('/'+ self.controller_side +'/arm_extend', Bool, queue_size=1)
        self.arm_length_pub = rospy.Publisher('/'+ self.controller_side +'/arm_length', Float64, queue_size=1)
        self.arm_shrink_pub = rospy.Publisher('/'+ self.controller_side +'/arm_shrink', Bool, queue_size = 1)
        self.arm_updown_pub = rospy.Publisher('/'+ self.controller_side +'/arm_updown', Float64, queue_size = 1)
        self.arm_front_pub = rospy.Publisher('/'+self.controller_side+'/arm_forward',Bool, queue_size = 1)
        self.arm_collsion_pub = rospy.Publisher('/'+self.controller_side+'/arm_collision',Bool, queue_size = 1)
        self.arm_cross_pub = rospy.Publisher('/'+self.controller_side+'/arm_cross',Bool, queue_size = 1)

        self.elbow_collision_pub = rospy.Publisher('/'+self.controller_side+'/elbow_collision',Bool, queue_size = 1)
        
        self.target_collosion_pub = rospy.Publisher('/'+self.controller_side+'/target_collision',Bool, queue_size = 1)

        self.current_above_desk_pub = rospy.Publisher('/'+self.controller_side+'/current_above_desk',Float64, queue_size = 1)


        self.move_collosion_pub = rospy.Publisher('/'+self.controller_side+'/move_collision',Bool, queue_size = 1)
        self.move_base_fast_pub = rospy.Publisher('/'+self.controller_side+'/base_fast_domain',Bool, queue_size = 10)

        self.object_frames = [
            {"frame": "front_desk", "length": 1.4, "heights": [0.85]},
            {"frame": "back_desk", "length": 1.0,"heights": [0.2]},
            {"frame": "right_back_desk", "length": 0.5,"heights": [0.6]},
            {"frame": "left_wall", "length": 0.5, "heights": [1.25, -0.9]},
            {"frame": "right_wall", "length": 0.5, "heights": [1.25, -0.9]}
        ]

                              
        self.arm_goal_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        rospy.Subscriber('/'+controller_side+'/compensate_pose', PoseStamped, self.__arm_input_callback)
        rospy.Subscriber('/move_base_target', Float32MultiArray, self.move_base_target_callback)

    def __arm_input_callback(self, msg):
        # self.prev_arm_goal_pose = copy.deepcopy(self.arm_goal_pose)
        self.arm_goal_pose['position'][0] = msg.pose.position.x
        self.arm_goal_pose['position'][1] = msg.pose.position.y
        self.arm_goal_pose['position'][2] = msg.pose.position.z
        self.arm_goal_pose['orientation'][0] = msg.pose.orientation.x
        self.arm_goal_pose['orientation'][1] = msg.pose.orientation.y
        self.arm_goal_pose['orientation'][2] = msg.pose.orientation.z
        self.arm_goal_pose['orientation'][3] = msg.pose.orientation.w


    def transform_pose_with_matrix(self, position, orientation, translation, rotation):
        # Convert quaternion to a 4x4 transformation matrix and extract the 3x3 rotation part
        rotation_matrix = quaternion_matrix(rotation)[:3, :3]
        
        # Rotate the position using the rotation matrix
        rotated_pos = np.dot(rotation_matrix, position)
        
        # Translate the position
        transformed_pos = rotated_pos + translation
        
        # Rotate the orientation using quaternion multiplication
        transformed_orientation = transformations.quaternion_multiply(rotation, orientation)
        
        return transformed_pos, transformed_orientation

    def move_base_target_callback(self, msg):
        self.x = msg.data[0]
        self.y = msg.data[1]
        self.z = 0.0
        


    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            object_collsion = False
            try:
                # Log the attempt to lookup transform
                #rospy.loginfo("Attempting to lookup transform from 'torso_lift_link' to 'arm_%s_tool_link'", self.controller_side)
                available = self.tfBuffer.can_transform('arm_' + self.controller_side + '_1_link', 'gripper_' + self.controller_side + '_grasping_frame', rospy.Time(0), rospy.Duration(5.0))
                
                if available:
                    #rospy.loginfo("Transform is available")
                    trans = self.tfBuffer.lookup_transform('torso_lift_link','gripper_' + self.controller_side + '_grasping_frame', rospy.Time(0))
                    translation = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
                    
                    compensate_trans = self.tfBuffer.lookup_transform('torso_lift_link', 'arm_' + self.controller_side + '_1_link', rospy.Time(0))
                    compensate_translation = np.array([compensate_trans.transform.translation.x, compensate_trans.transform.translation.y, compensate_trans.transform.translation.z])


                    coll = self.tfBuffer.lookup_transform('odom', 'gripper_' + self.controller_side + '_grasping_frame', rospy.Time(0))
                    collsion = np.array([coll.transform.translation.x, coll.transform.translation.y, coll.transform.translation.z])

                    
                    elbow = self.tfBuffer.lookup_transform('odom', 'arm_' + self.controller_side + '_4_link', rospy.Time(0))
                    elbow_pos = np.array([elbow.transform.translation.x, elbow.transform.translation.y, elbow.transform.translation.z])
                    
                    
                    gripper_to_arm_tool_link = self.tfBuffer.lookup_transform('arm_' + self.controller_side + '_tool_link', 'gripper_' + self.controller_side + '_grasping_frame', rospy.Time(0))
                    gripper_to_arm_tool_link_pos = np.array([gripper_to_arm_tool_link.transform.translation.x, gripper_to_arm_tool_link.transform.translation.y, gripper_to_arm_tool_link.transform.translation.z])
                    gripper_to_arm_tool_link_quat = np.array([gripper_to_arm_tool_link.transform.rotation.x, gripper_to_arm_tool_link.transform.rotation.y, gripper_to_arm_tool_link.transform.rotation.z, gripper_to_arm_tool_link.transform.rotation.w])



                    base_to_odom = self.tfBuffer.lookup_transform('odom','torso_lift_link', rospy.Time(0))
                    base_to_odom_pos = np.array([base_to_odom.transform.translation.x, base_to_odom.transform.translation.y, base_to_odom.transform.translation.z])
                    base_to_odom_quat = np.array([base_to_odom.transform.rotation.x, base_to_odom.transform.rotation.y, base_to_odom.transform.rotation.z, base_to_odom.transform.rotation.w])
                    base_to_odom_matrix = quaternion_matrix(base_to_odom_quat)[:3, :3]

                    arm_goal_pose_odom = self.transform_pose_with_matrix(self.arm_goal_pose['position'], self.arm_goal_pose['orientation'], base_to_odom_pos, base_to_odom_quat)

                    # Transform the gripper position and orientation from arm tool link frame to odom frame
                    rotated_pos, rotated_quat = self.transform_pose_with_matrix(gripper_to_arm_tool_link_pos, gripper_to_arm_tool_link_quat, arm_goal_pose_odom[0], arm_goal_pose_odom[1])

                    next_target_collision = {
                        'position': rotated_pos,
                        'orientation': rotated_quat
                    }
                    
                    # if self.controller_side == "right":
                    #      rospy.loginfo("next target collision: %s", next_target_collision)
                    
                    
                    
                    

                    #current arm collsion
                    #This to detect whether the arm is colliding and control arm_controlled movement
                    coll_msg = Bool()
                    coll_msg.data = False
                    if (collsion[1] > 1.19 and collsion[0] >0) or collsion[1] < -1.02: # wall collsion
                        coll_msg.data = True
                        #rospy.logwarn("wall collsion")
                    if collsion[0] > 1.1 and collsion[2]<0.72: #front desk collsion
                        coll_msg.data = True
                        #rospy.logwarn("front desk collsion")
                    if collsion[0] < -0.75 and collsion[2]<0.42 and collsion[1]>-0.45: # back desk collsion
                        coll_msg.data = True
                        #rospy.logwarn("back desk collsion")
                    if collsion[0] < -0.7 and collsion[2]<0.67 and collsion[1]<-0.45: # right back desk collsion
                        coll_msg.data = True
                        #rospy.logwarn("right back desk collsion")

                    #current arm above desk 
                    # This to detect whether the arm is above the desk and allow the torso to move up and down
                    current_above_task = 0.0
                    if collsion[0] < -0.75 and collsion[2] < 0.49 and collsion[1]>-0.45: # back desk collsion
                        current_above_task = 1.0
                        #rospy.logwarn("current above back desk")
                    if collsion[0] < -0.7 and collsion[2] < 0.7 and collsion[1]<-0.45: # right back desk collsion
                        current_above_task = 1.0
                        rospy.logwarn("current above right back desk")
                    if collsion[0] > 1.1 and collsion[2] < 0.78: # front desk collsion
                        current_above_task = 1.0
                        #rospy.logwarn("current above front desk")
                    if (collsion[0] > -0.69 and collsion[0] < -0.27) and collsion[1] > 0.77: # shel area
                        current_above_task = 2.0
                        #rospy.logwarn("current above shelf area")

                    

                    #arm_next_target_collsion
                    #This to detect whether the arm next status will collsion with the wall or desk
                    target_collision = Bool()
                    target_collision.data = False
                    if (next_target_collision['position'][1] > 1.12 and next_target_collision['position'][0] >0) or next_target_collision['position'][1] < -1.02: # wall collsion
                        target_collision.data = True
                        #rospy.logwarn("target wall collsion")
                    if next_target_collision['position'][0] > 1.1 and next_target_collision['position'][2]<0.72: #front desk collsion
                        target_collision.data = True
                        #rospy.logwarn("target front desk collsion")
                    if next_target_collision['position'][0] < -0.75 and next_target_collision['position'][2]<0.43 and next_target_collision['position'][1]>-0.45: # back desk collsion
                        target_collision.data = True
                        #rospy.logwarn("target back desk collsion")
                    if next_target_collision['position'][0] < -0.7 and next_target_collision['position'][2]<0.7 and next_target_collision['position'][1]<-0.45: # right back desk collsion
                        target_collision.data = True
                        #rospy.logwarn("target right back desk collsion")
                    
                    
                    
                    #elbow collsion
                    #This to detect whether the elbow is colliding with the wall or desk only showing color change
                    elbow_collision = Bool()
                    elbow_collision.data = False
                    if (elbow_pos[1] > 1.19 and elbow_pos[0]>0) or elbow_pos[1] < -1.02: # wall collsion
                        elbow_collision.data = True
                        #rospy.logwarn("elbow wall collsion")
                    if elbow_pos[0] > 1.1 and elbow_pos[2]<0.72: #front desk collsion
                        elbow_collision.data = True
                        #rospy.logwarn("elbow front desk collsion")
                    if elbow_pos[0] < -0.75 and elbow_pos[2]<0.42 and elbow_pos[1]>-0.45: # back desk collsion
                        elbow_collision.data = True
                        #rospy.logwarn("elbow back desk collsion")
                    if elbow_pos[0] < -0.75 and elbow_pos[2]<0.6 and elbow_pos[1]<-0.45: # right back desk collsion
                        elbow_collision.data = True
                        #rospy.logwarn("elbow right back desk collsion")
                    
                    # move collision
                    # This to detect whether the arm will collsion with the wall or desk and allow keyboard moving
                    next_movement = np.array([self.x, self.y, self.z])
                    rotated_pos = np.dot(base_to_odom_matrix, next_movement)
                    #rospy.loginfo(f"rotated_pos: {rotated_pos}")
                    arm_move_collsion = collsion + rotated_pos
                    #rospy.loginfo(f"arm move collsion: {arm_move_collsion}")
                    move_collsion = Bool()
                    move_collsion.data = False
                    if (arm_move_collsion[1] > 1.21 and arm_move_collsion[0] >0) or arm_move_collsion[1] < -1.02: # wall collsion
                        move_collsion.data = True
                        #rospy.logwarn("move wall collsion")
                    if arm_move_collsion[0] > 1.1 and arm_move_collsion[2]<0.72: #front desk collsion
                        move_collsion.data = True
                        #rospy.logwarn("move front desk collsion")
                    if arm_move_collsion[0] < -0.75 and arm_move_collsion[2]<0.42 and arm_move_collsion[1]>-0.45: # back desk collsion
                        move_collsion.data = True
                        #rospy.logwarn("move back desk collsion")
                    if arm_move_collsion[0] < -0.75 and arm_move_collsion[2]<0.67 and arm_move_collsion[1]<-0.45: # right back desk collsion
                        move_collsion.data = True
                        #rospy.logwarn("move right back desk collsion")
                    
                    wall = Bool()
                    front_desk = Bool()
                    back_desk = Bool()
                    right_back_desk = Bool()

                    arm_move_collsion = collsion + rotated_pos*20
                    if (arm_move_collsion[1] < 1.21 or arm_move_collsion[0] < 0) and arm_move_collsion[1] > -1.02: # wall collsion
                        wall = False
                        #rospy.logwarn("move wall collsion disalarm")
                    if arm_move_collsion[0] < 1.1 or arm_move_collsion[2]>0.72: #front desk collsion
                        front_desk = False
                        #rospy.logwarn("move front desk collsion disalarm")
                    if arm_move_collsion[0] > -0.75 or arm_move_collsion[2]>0.42 or arm_move_collsion[1]>-0.45: # back desk collsion
                        back_desk = False
                        #rospy.logwarn("move back desk collsion disalarm")
                    if arm_move_collsion[0] > -0.75 or arm_move_collsion[2]>0.67 or arm_move_collsion[1]<-0.45: # right back desk collsion
                        right_back_desk = False
                        #rospy.logwarn("move right back desk collsion disalarm")

                    if (not wall) and (not front_desk) and (not back_desk) and (not right_back_desk):
                        move_collsion.data = False

                    #base_fast_domain
                    base_fast_domain = Bool()
                    base_fast_domain.data = True
                    if base_to_odom_pos[0] > 0.425 or base_to_odom_pos[0] < -0.24 or base_to_odom_pos[1] > 0.19 or base_to_odom_pos[1] < -0.3:
                        base_fast_domain.data = False
                        #rospy.logwarn("base leave fast domain")


                    # Calculate the norm (magnitude) of the translation vector
                    relative_translation = translation - compensate_translation
                    ee_norm = LA.norm(translation)
                    xy_ee_norm = LA.norm(relative_translation[:2])
                    ext_msg = Bool()
                    #rospy.loginfo(f"xy norm is:{xy_ee_norm}")
                    #ext_msg.data = xy_ee_norm > self.extend_limit #xy distance 

                    if self.controller_side == "right":
                        ext_msg.data = relative_translation[1]< -0.24
                        
                    if self.controller_side == "left":
                        ext_msg.data = relative_translation[1]> 0.24
                        
                    
                    front_msg = Bool()
                    front_msg.data = relative_translation[0] > self.extend_limit
                    #rospy.loginfo(f"x is:{relative_tranlation[0]}")
                    

                    srk_msg = Bool()
                    srk_msg.data = relative_translation[0] < self.shrink_limit

                    updown_msg = 0.0
                    #rospy.logwarn(f"current z:{relative_tranlation[2]}")
                    if relative_translation[2] > self.up_limit:
                        updown_msg = 1.0
                    if relative_translation[2] <self.down_limit:
                        updown_msg = -1.0


                    cross_msg = Bool()
                    if self.controller_side == "right":
                        cross_msg.data = translation[1] > -0.09
                        
                    if self.controller_side == "left":
                        cross_msg.data = translation[1] < 0.09
                        
                    #rospy.loginfo("Publishing arm extend: %s", ext_msg.data)
                    #rospy.loginfo("Publishing arm length: %f", ee_norm)

                    ###----------------- calculating the object facing -----------------###




                    ###----------------- Publish the arm status -----------------###
                    self.arm_extend_pub.publish(ext_msg)
                    self.arm_length_pub.publish(ee_norm)
                    self.arm_shrink_pub.publish(srk_msg)
                    self.arm_updown_pub.publish(updown_msg)
                    self.arm_front_pub.publish(front_msg)
                    self.arm_collsion_pub.publish(coll_msg)
                    self.arm_cross_pub.publish(cross_msg)
                    self.target_collosion_pub.publish(target_collision)
                    self.elbow_collision_pub.publish(elbow_collision)
                    self.move_collosion_pub.publish(move_collsion)
                    self.move_base_fast_pub.publish(base_fast_domain)
                    self.current_above_desk_pub.publish(current_above_task)
                else:
                    rospy.logwarn("Transform not available yet")
                
                rate.sleep()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn("TF Exception: %s", e)
                rate.sleep()
        

## -------------- Main Function -------------- ## 
def main():
    try:
        args = rospy.myargv(argv=sys.argv)
        controller_side = args[1]

        # rospy.init_node('tiago_arm'+controller_side+'_position_control')
        rospy.init_node('check_'+controller_side+'ee_loc')
        app = TiagoArmEEChecker(controller_side=controller_side)
        app.run()

    except rospy.ROSInterruptException:
        rospy.logwarn("something wrong")
        pass

if __name__ == '__main__':
    main()