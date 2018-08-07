# Arm
ARM_CLIENT_TOPIC = '/hsrb/arm_trajectory_controller/follow_joint_trajectory'
ARM_JOINTS = ['arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
# Head
HEAD_CLIENT_TOPIC = '/hsrb/head_trajectory_controller/follow_joint_trajectory'
HEAD_STATE_TOPIC = '/hsrb/head_trajectory_controller/state'
HEAD_JOINTS = ['head_pan_joint', 'head_tilt_joint']
FULL_PAN = 1.16  # Minimum change required to pan completely away
FULL_TILT = 0.9  # Minimum change required to tilt completely away
# Speech
SPEECH_TOPIC = '/talk_request'
SPEECH_CLIENT_TOPIC = '/talk_request_action'
# Base
MOVE_BASE_TOPIC = '/move_base/move'
OMNI_BASE_CLIENT_TOPIC = '/hsrb/omni_base_controller/follow_joint_trajectory'
OMNI_BASE_JOINTS = ['odom_x', 'odom_y', 'odom_t']
# Gripper
GRIPPER_CLIENT_TOPIC = '/hsrb/gripper_controller/grasp'