alias tux='ssh tux@Roman'
alias hans='ssh hans@Hans-Laptop-Linux'
alias wilson='ssh kelvin@VPCEB2M1E'

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

alias base_forward='rostopic pub /cmd_mobile_position std_msgs/Float32 1'
alias base_backward='rostopic pub /cmd_mobile_position std_msgs/Float32 -- -1'
alias turn_left='rostopic pub /cmd_mobile_turn std_msgs/Float32 0.785398163'
alias turn_right='rostopic pub /cmd_mobile_turn" std_msgs/Float32 -- -0.785398163'

alias focus_face_on='rosservice call /set_focus_face true'
alias focus_face_off='rosservice call /set_focus_face false'

alias wake_up='rostopic pub /processedSpeechTopic audio_processing/speech "wake up" 0'
alias sleep='rostopic pub /processedSpeechTopic audio_processing/speech "sleep" 0'
alias juice='rostopic pub /processedSpeechTopic audio_processing/speech "juice" 0'
alias cola='rostopic pub /processedSpeechTopic audio_processing/speech "cola" 0'
alias happy='rostopic pub /cmd_emotion std_msgs/UInt8 1'
alias sad='rostopic pub /cmd_emotion std_msgs/UInt8 2'
alias surprised='rostopic pub /cmd_emotion std_msgs/UInt8 3'
alias neutral='rostopic pub /cmd_emotion std_msgs/UInt8 0'




