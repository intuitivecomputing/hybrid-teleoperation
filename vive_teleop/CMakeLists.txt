cmake_minimum_required(VERSION 3.0.2)
project(vive_teleop)

find_package(catkin REQUIRED COMPONENTS

)
catkin_package(
  CATKIN_DEPENDS actionlib)

catkin_install_python(PROGRAMS scripts/gamepad_teleop.py
                               scripts/incrementer_server.py
                               scripts/vive_teleop.py
                               scripts/vive_teleop_v2.py
                               scripts/vive_teleop_continue.py
				scripts/vive_teleop_continue_notouchpad.py
                               scripts/ee_publisher.py
                               scripts/ee_publisher_fall.py
				scripts/ee_publisher_continue.py
                               scripts/data_recorder.py
                               scripts/data_recorder_fall.py
                               scripts/data_recorder_continue.py
                               scripts/vive_pose_mapping.py
                               scripts/vive_pose_mapping_fall.py
                               scripts/vive_pose_mapping_continue.py
                               scripts/tiago_arm_position_control.py
                               scripts/tiago_arm_gamepad_control.py
                               scripts/tiago_arm_position_control_fall.py
				scripts/tiago_arm_position_control_continue.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(dir config launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
