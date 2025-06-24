#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""
This script is used for teleoperating the Tiago++ robots arms.

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

import sys
import rospy
import math
import numpy as np
from numpy import linalg as LA
import csv
import datetime
import time
import tf
import actionlib
import copy
import os

from geometry_msgs.msg import Twist, PoseStamped, TwistStamped, Pose
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64, Bool
from std_srvs.srv import Empty  # Import Empty service message
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from controller_manager_msgs.srv import SwitchController
from moveit_msgs.msg import MoveItErrorCodes

from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from teleop_tools_msgs.msg import IncrementAction as TTIA
from teleop_tools_msgs.msg import IncrementGoal as TTIG


from trac_ik_python.trac_ik import IK


class TiagoArmPositionControl():
    def __init__(self,
                 controller_side='right',
                 sim=True):

        ## -------------- Variables -------------- ##
        self.sim = sim
        self.controller_side = controller_side
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.expanduser(f"~/RAL2024/teleop_data/{controller_side}_arm_{timestamp}.csv")
        self.start_time = rospy.Time.now().to_sec()

        ### ------- TF ------- ###
        self.listener = tf.TransformListener()
        self.tf_br = tf.TransformBroadcaster()

        ### ------- Arm ------- ###
        self.urdf_str = rospy.get_param('/robot_description')
        # self.arm_base_link = "base_footprint"
        self.arm_base_link = "torso_lift_link"
        self.arm_ee_link = "arm_"+ controller_side + "_tool_link"
        self.arm_ik_solver = IK(self.arm_base_link, self.arm_ee_link)
        self.arm_joint_states = [0.0] * self.arm_ik_solver.number_of_joints
        self.arm_goal_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        ### ------- Torso ------- ###
        self.torso_joint = 0.2

        ## -------------- Flags -------------- ##
        self.activated = False
        self.homing = False
        self.trigger_pressed = False
        self.gripper_pressed = False
        self.home_status = False
        self.collsion = False


        ## -------------- ROS Publishers -------------- ##
        self.target_pose_pub = rospy.Publisher('/arm_'+ controller_side +'_target_pose', Pose, queue_size=1)
        self.arm_extend_pub = rospy.Publisher('/'+ controller_side +'/arm_extend', Bool, queue_size=1)
        self.home_pub = rospy.Publisher('/'+ controller_side +'/home', Bool, queue_size=1)
        self.gripper_state_pub = rospy.Publisher('/'+ controller_side +'/gripper_state', Bool, queue_size=1)

        ## -------------- ROS Action Clients ------------ ##
        if sim:
            # self.arm_client = actionlib.SimpleActionClient('/arm_'+controller_side+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            self.arm_traj_pub = rospy.Publisher('/arm_'+ controller_side +'_controller/command', JointTrajectory, queue_size=1)

        else:
            # self.arm_client = actionlib.SimpleActionClient('/safe_arm_'+controller_side+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            self.arm_traj_pub = rospy.Publisher('/arm_'+ controller_side +'_controller/safe_command', JointTrajectory, queue_size=1)

        rospy.wait_for_service('/parallel_gripper_'+controller_side+'_controller/grasp')
        self.gripper_client = rospy.ServiceProxy('/parallel_gripper_'+controller_side+'_controller/grasp', Empty)
        self.gripper_client_open = actionlib.SimpleActionClient('/parallel_gripper_'+controller_side+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)

        rospy.sleep(0.1)

        ## -------------- ROS Subscribers -------------- ##
        rospy.Subscriber('/'+controller_side+'/compensate_pose', PoseStamped, self.__arm_input_callback)
        rospy.Subscriber('/'+controller_side+'/target_collision', Bool, self.__collsion_callback)
        if controller_side == "right":
            rospy.Subscriber('/Right_Buttons', Joy, self.__gripper_control)

        elif controller_side == "left":
            rospy.Subscriber('/Left_Buttons', Joy, self.__gripper_control)

        rospy.Subscriber('/joint_states', JointState, self.__joint_states_callback)
        rospy.Subscriber('/'+controller_side+'/robot_activation', Float64, self.__robot_activation_callback)



        rospy.sleep(0.2)


    ## -------------- Callback Functions -------------- ## 

    def __collsion_callback(self, msg):
        self.collsion = msg.data

    def __robot_activation_callback(self, msg):
        # rospy.logwarn("%s", msg.data)

        if msg.data == 2.0 and self.activated == False and not self.homing:
            self.activated = True
            self.home_status = False
            rospy.logwarn(self.controller_side + "arm activated")

        if msg.data == 0.0 and self.activated == True and not self.homing:
            self.activated = False
            rospy.logwarn(self.controller_side + "arm stopped")



    # def __get_arm_left_joint_states(self):
    def __joint_states_callback(self, msg):

        self.torso_joint = msg.position[20]
        if self.arm_ik_solver.number_of_joints >= 7:
            if self.controller_side == "left":
                self.arm_joint_states = [msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5], msg.position[6]] 

            elif self.controller_side == "right":
                self.arm_joint_states = [msg.position[7], msg.position[8], msg.position[9], msg.position[10],
                                          msg.position[11], msg.position[12], msg.position[13]] 

            if self.arm_ik_solver.number_of_joints == 8:
                self.arm_joint_states = self.arm_joint_states.insert(0, torso_joint)

    def __arm_input_callback(self, msg):
        # self.prev_arm_goal_pose = copy.deepcopy(self.arm_goal_pose)
        self.arm_goal_pose['position'][0] = msg.pose.position.x
        self.arm_goal_pose['position'][1] = msg.pose.position.y
        self.arm_goal_pose['position'][2] = msg.pose.position.z
        self.arm_goal_pose['orientation'][0] = msg.pose.orientation.x
        self.arm_goal_pose['orientation'][1] = msg.pose.orientation.y
        self.arm_goal_pose['orientation'][2] = msg.pose.orientation.z
        self.arm_goal_pose['orientation'][3] = msg.pose.orientation.w


    def __gripper_control(self, joy_msg):
        ## if trigger pressed
        if(joy_msg.buttons[1] == 1):
            if not self.trigger_pressed:  # If trigger was not pressed before
                if not self.gripper_pressed:  # If gripper is currently closed
                    self.gripper_client()
                   
                    self.gripper_pressed = True
                    self.gripper_state_pub.publish(self.gripper_pressed)
                else:
                    gripper_position = 0.075  # open the gripper
                    # # Create a trajectory point for the gripper
                    gripper_point = JointTrajectoryPoint()
                    gripper_point.positions = [gripper_position]
                    gripper_point.velocities = []
                    gripper_point.accelerations = []
                    gripper_point.effort = []
                    gripper_point.time_from_start = rospy.Duration(0.5)  # Adjust the duration as needed

                    # Create and send the gripper goal
                    gripper_goal = FollowJointTrajectoryGoal()
                    gripper_goal.trajectory.points = [gripper_point]
                    gripper_goal.trajectory.header.stamp = rospy.Time.now()

                    gripper_goal.trajectory.joint_names = ["gripper_"+self.controller_side+"_left_finger_joint", "gripper_"+self.controller_side+"_right_finger_joint"]
                    self.gripper_client_open.send_goal(gripper_goal)

                    self.gripper_pressed = False
                    self.gripper_state_pub.publish(self.gripper_pressed)



            self.trigger_pressed = True  # Update trigger state

        else:
            self.trigger_pressed = False  # Update trigger state

        ## if menu button pressed
        if (joy_msg.buttons[0] == 1):
            self.arm_homing()


    ## -------------- Helper Functions -------------- ## 

    def __get_arm_transformation(self):
        try:
            (trans, rot) = self.listener.lookupTransform(self.arm_base_link, self.arm_ee_link, rospy.Time(0))
            pose_message = Pose()
            pose_message.position.x = trans[0]
            pose_message.position.y = trans[1]
            pose_message.position.z = trans[2]

            pose_message.orientation.x = rot[0]
            pose_message.orientation.y = rot[1]
            pose_message.orientation.z = rot[2]
            pose_message.orientation.w = rot[3]

            return pose_message
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None


    def __compose_pose_message(self, target_pose):
        """
        target_pose: dict
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0])
        
        """

        # NOTE: These two checks might not be needed, check function usage.
        if not isinstance(target_pose, dict):
            raise TypeError('target_pose is not a dictionary.')

        for key in ['position', 'orientation']:
            if key not in target_pose:
                raise KeyError(f'key {key} not found in target_pose.')

        pose_message = Pose()
        pose_message.position.x = target_pose['position'][0]
        pose_message.position.y = target_pose['position'][1]
        pose_message.position.z = target_pose['position'][2]

        pose_message.orientation.x = target_pose['orientation'][0]
        pose_message.orientation.y = target_pose['orientation'][1]
        pose_message.orientation.z = target_pose['orientation'][2]
        pose_message.orientation.w = target_pose['orientation'][3]

        return pose_message


    def publish_arm_trajectory(self):
        vel_gain = 1  # Adjust the maximum velocity as needed

        if self.activated:  
            # Solve IK
            if self.arm_joint_states is not None and self.collsion == False:

                target_joint_angles = self.arm_ik_solver.get_ik(self.arm_joint_states,
                  self.arm_goal_pose['position'][0], self.arm_goal_pose['position'][1], self.arm_goal_pose['position'][2],
                  self.arm_goal_pose['orientation'][0], self.arm_goal_pose['orientation'][1], self.arm_goal_pose['orientation'][2], self.arm_goal_pose['orientation'][3])

                self.tf_br.sendTransform((self.arm_goal_pose['position'][0], self.arm_goal_pose['position'][1], self.arm_goal_pose['position'][2]),
                                        (self.arm_goal_pose['orientation'][0], self.arm_goal_pose['orientation'][1], self.arm_goal_pose['orientation'][2], self.arm_goal_pose['orientation'][3]),
                                         rospy.Time.now(), self.controller_side + "_ee_goal", self.arm_base_link)

                # Create a JointTrajectory message
                if target_joint_angles is not None:
                    if LA.norm(np.array(target_joint_angles) - np.array(self.arm_joint_states)) < np.pi/3:

                        joint_trajectory_goal = JointTrajectory()
                        joint_trajectory_goal.header.stamp = rospy.Time.now() # + rospy.Duration(0.3)
                        joint_trajectory_goal.joint_names = list(self.arm_ik_solver.joint_names)

                        trajectory_point = JointTrajectoryPoint()
                        trajectory_point.positions = target_joint_angles

                        trajectory_point.velocities = [(end - start) * vel_gain for start, end in zip(self.arm_joint_states, target_joint_angles)]
                        trajectory_point.time_from_start = rospy.Duration(0.6)


                        joint_trajectory_goal.points.append(trajectory_point)

                        self.arm_traj_pub.publish(joint_trajectory_goal)

                        self.target_pose_pub.publish(self.__compose_pose_message(self.arm_goal_pose))

                        # ext_msg = Bool()
                        # ext_msg.data = LA.norm(self.arm_goal_pose['position']) > 0.8
                        # rospy.logwarn("norm: %s", LA.norm(self.arm_goal_pose['position']))
                        # self.arm_extend_pub.publish(ext_msg)

                    else:
                        pass
                        # rospy.logwarn("large norm: %s", LA.norm(np.array(target_joint_angles) - np.array(self.arm_joint_states)))


                else:
                    pass
                    # rospy.logwarn("unable to solve ik")


    def arm_homing(self):
        ## stop teleop during arm homing
        self.activated = False
        self.homing = True

        rospy.logwarn("%s arm moving to home position...", self.controller_side)
        # motion_name = "tele_home_" + self.controller_side
        motion_name = "tele_home_" + self.controller_side + "_v2"

        # motion_name = "tele_home"
        home_goal = PlayMotionGoal(motion_name = motion_name, skip_planning = False)
        self.play_motion_client.send_goal(home_goal)
        self.play_motion_client.wait_for_result()

        arm_home_pose = self.__get_arm_transformation()
        self.target_pose_pub.publish(arm_home_pose)
        self.tf_br.sendTransform((arm_home_pose.position.x, arm_home_pose.position.y, arm_home_pose.position.z),
                                (arm_home_pose.orientation.x, arm_home_pose.orientation.y, arm_home_pose.orientation.z, arm_home_pose.orientation.w),
                                 rospy.Time.now(), self.controller_side + "_ee_goal", self.arm_base_link)

        rospy.logwarn("%s arm moved to home", self.controller_side)
        self.homing = False
        self.home_status = True
        # self.activated = True


    def save_to_csv(self):
        directory = os.path.dirname(self.filename)
        if not os.path.exists(directory):
            os.makedirs(directory)
        arm_current_pose = self.__get_arm_transformation()
        t = rospy.Time.now().to_sec() - self.start_time
        with open(self.filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([t, arm_current_pose.position.x, arm_current_pose.position.y, arm_current_pose.position.z,
                            arm_current_pose.orientation.x, arm_current_pose.orientation.y, arm_current_pose.orientation.z, arm_current_pose.orientation.w])


    def run(self):
        self._running = True
        counter = 0

        while self._running and not rospy.is_shutdown():

            self.publish_arm_trajectory()
            self.home_pub.publish(self.home_status)
            rospy.sleep(0.005)
            # counter += 1

## -------------- Node Destructor -------------- ## 

def node_shutdown():
    print(f'\nArm position control has been shut down...\n')


## -------------- Main Function -------------- ## 
def main():
    try:
        args = rospy.myargv(argv=sys.argv)
        controller_side = args[1]
        sim = args[2]
        if sim == "true":
            sim_param=True
        elif sim == "false":
            sim_param = False
        rospy.init_node('tiago_arm'+controller_side+'_position_control')
        app = TiagoArmPositionControl(controller_side=controller_side, sim=sim_param)
        app.run()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

