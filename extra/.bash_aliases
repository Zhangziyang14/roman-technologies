alias tux='ssh tux@Roman'
alias hans='ssh hans@Hans-Laptop-Linux'
alias wilson='ssh kelvin@???'

alias launch='roslaunch nero_launch nero_complete.launch'
alias launch_focus_face='roslaunch nero_launch nero_focus_face.launch'
alias rviz='rosrun rviz rviz'

alias gripper_open='rostopic pub /cmd_gripper_state std_msgs/Bool true'
alias gripper_close='rostopic pub /cmd_gripper_state std_msgs/Bool false'
alias ping_open='rostopic pub /cmd_gripper std_msgs/Bool true'
alias ping_close='rostopic pub /cmd_gripper std_msgs/Bool false'

alias head_normal='rostopic pub /cmd_head_position head/PitchYaw 0 0'
alias head_detect='rostopic pub /cmd_head_position head/PitchYaw 0.4 0'
alias head_face='rostopic pub /cmd_head_position head/PitchYaw -- -0.2 0'
alias head_sleep='rostopic pub /cmd_head_position head/PitchYaw 0.6 0'

alias arm_down='rostopic pub /cmd_arm_position geometry_msgs/Pose -- "[-1, 0, -1]" "[0, 0, 0, 0]"'
alias arm_up='rostopic pub /cmd_arm_position geometry_msgs/Pose -- "[0, 0, -0.15]" "[0, 0, 0, 0]"'

alias focus_face_on='rosservice call /set_focus_face true'
alias focus_face_off='rosservice call /set_focus_face false'

