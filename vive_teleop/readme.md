data recorder continue 

mains structure
task(1,2,3,4)->method(ego/baseline/combined)->test(number)

robot_to_target: Get the coordinates and quarterions with respect to each objects

robot_states:
	time: current timestamp
	left_arm_state: whether the robot arm is being controlled, 2 means control, 0 means pause
	left_gripper_state: 0 means the gripper is open; 1 means is closing
	left_arm_xyz,qxqyqzqw: coordinates of the gripper_left_grasping_frame with respect to torso_lift_link
	Same for the right
	head_pan/head/tilt torso_lift base_linear_xyz is the meaning of its name
	left_target_collision: check the next movement of the arm(comes from vive), if the next move will cause the collision, it will be 1 otherwise it will be 0
	left_move_collsion: check the touchpad movement if the next move will cause the collision, it will be 1 otherwise it will be 0
	left_elbow_collsion: check whether the elbow is too close to obsacle, 1 is close 0 is not
	left_current_above_desk: check whether the arm is too close to table in Z direction. if 1 means torso cannot move down; if 2 means torso cannot move up and down(in shelf); 0 no constraints
	Same for the right
	Base_fast_domain: 1 is in; 0 is out
	
Base_trajectory: base coordinates with respect to odom

controller_states(record base line):
	right_hand_xyz,qxqwqyqz: right_vive controller status.
	Same for left and head
	right_axes_1:records the side pressing extent very positive means rotate right/ very negative means rotate left
	right_axes_2:records the front pressing extent very positive means torso up/ very negative means torso down
	right_axes_3: always 0 useless
	right_buttons_1: Pressed will be 1 and means Homing
	right_buttons_2: Pressed will be 1 means change control state(pause to control or control to pause)
	right_buttons_3: touchpad is pressed
	touch_buttons_4: gripper is pressed, meaning change states(open to close and close to open)
	Same for left
	left_axes_1:records the side pressing extent very positive means translation right/ very negative means translation left
	left_axes_2:records the front pressing extent very positive means translation front/ very negative means translation back
	
egocentric_states(record notoucpad):
	Same for base position,
	right_arm_extend: right_arm extends right boundary means 1 otherwise 0
	right_arm_shrink: right_arm shrink back boudary means 1 otherwise 0
	right_arm_forward: right_arm forward front boundary means 1 otherwise 0 
	right_arm_cross: right_arm cross center boundary means 1 otherwise 0
	right_arm_updown: right_arm reach top means 1 and down means -1 otherwise 0
	Same for left
	Head_states: 1.0 means torso up, -1.0 means torso down; 2.0 means rotate left; -2.0 means rotate right; otherwise 0
	
Combined_states(record task4): All the above two control added up

