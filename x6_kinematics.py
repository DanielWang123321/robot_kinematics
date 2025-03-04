import numpy as np
import yaml
from scipy.optimize import least_squares

# Load kinematics parameters from YAML file
def load_kinematics_params(yaml_file):
    with open(yaml_file, 'r') as file:
        params = yaml.safe_load(file)
    return params

# Load parameters from YAML file
kin_params = load_kinematics_params('x6_kine_para.yaml')

# Extract joint limits and DH parameters
joint_limits = kin_params['joint_limits']
dh_params = [
    kin_params['dh_params']['joint1'],
    kin_params['dh_params']['joint2'],
    kin_params['dh_params']['joint3'],
    kin_params['dh_params']['joint4'],
    kin_params['dh_params']['joint5'],
    kin_params['dh_params']['joint6']
]

# Tool Coordinate System (TCP) Transformation
T_tool = np.array(kin_params['tool']['transform'])

def dh_transform(alpha, a, d, theta):
    """Calculate Denavit-Hartenberg transformation matrix"""
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -d * np.sin(alpha)],
        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), d * np.cos(alpha)],
        [0, 0, 0, 1]
    ])

def forward_kinematics_internal(joint_angles):
    """Internal forward kinematics calculation (assumes radians)"""
    T = np.eye(4)
    for i, (alpha, a, d, theta_offset) in enumerate(dh_params):
        theta = joint_angles[i] + theta_offset
        T = T @ dh_transform(alpha, a, d, theta)
    T = T @ T_tool
    return T

def pose_error(joint_angles, target_pose):
    """Calculate pose error for optimization"""
    current_pose = forward_kinematics_internal(joint_angles)
    
    current_position = current_pose[:3, 3]
    target_position = target_pose[:3]
    
    r11, r12, r13 = current_pose[0, :3]
    r21, r22, r23 = current_pose[1, :3]
    r31, r32, r33 = current_pose[2, :3]
    
    current_roll = np.degrees(np.arctan2(r32, r33))
    current_pitch = np.degrees(np.arctan2(-r31, np.sqrt(r32**2 + r33**2)))
    current_yaw = np.degrees(np.arctan2(r21, r11))
    
    current_orientation = np.array([current_roll, current_pitch, current_yaw])
    target_orientation = target_pose[3:]
    
    position_error = current_position - target_position
    orientation_error = current_orientation - target_orientation
    
    for i in range(3):
        while orientation_error[i] > 180:
            orientation_error[i] -= 360
        while orientation_error[i] < -180:
            orientation_error[i] += 360
    
    joint_minimization = np.abs(joint_angles)
    joint_weight = 0.1
    
    return np.concatenate([
        position_error,
        orientation_error / 10,
        joint_minimization * joint_weight
    ])

def inverse_kinematics_internal(target_pose, initial_guess=None):
    """Internal inverse kinematics calculation"""
    if initial_guess is None:
        initial_guess = np.zeros(6)
    
    initial_guesses = [
        initial_guess,
        np.zeros(6),
        np.array([0, -np.pi/4, -np.pi/4, 0, 0, 0]),
        np.array([np.pi/2, 0, 0, 0, np.pi/2, 0]),
        np.array([-np.pi/2, 0, 0, 0, -np.pi/2, 0])
    ]
    
    target_pose_internal = target_pose.copy()
    target_pose_internal[3:] = np.radians(target_pose_internal[3:])
    
    best_solution = None
    best_error = float('inf')
    best_joint_sum = float('inf')
    
    for guess in initial_guesses:
        result = least_squares(
            lambda x: pose_error(x, target_pose),
            guess,
            bounds=([joint_limits[f'joint{i+1}'][0] for i in range(6)],
                    [joint_limits[f'joint{i+1}'][1] for i in range(6)]),
            method='trf',
            ftol=1e-3,
            max_nfev=1000
        )
        
        joint_angles = result.x
        final_error = pose_error(joint_angles, target_pose)
        position_error = np.linalg.norm(final_error[:3])
        orientation_error = np.linalg.norm(final_error[3:6])
        total_error = position_error + orientation_error
        joint_sum = np.sum(np.abs(joint_angles))
        
        if total_error < 1.0 and (best_solution is None or joint_sum < best_joint_sum):
            best_solution = joint_angles
            best_error = total_error
            best_joint_sum = joint_sum
        elif best_solution is None or total_error < best_error:
            best_solution = joint_angles
            best_error = total_error
            best_joint_sum = joint_sum
    
    return best_solution

def x6_get_fk(angles, input_is_radian=False, output_is_radian=False):
    """Calculate forward kinematics for xArm 6.
    
    Args:
        angles: List of 6 joint angles
        input_is_radian: If True, input angles are in radians; if False, in degrees
        output_is_radian: If True, output orientation is in radians; if False, in degrees
    
    Returns:
        List of [x, y, z, roll, pitch, yaw] representing TCP pose
    """
    if len(angles) != 6:
        raise ValueError("Input angles must be a list of 6 values")
    
    angles = np.array(angles)
    if not input_is_radian:
        angles = np.radians(angles)
    
    # Check joint limits
    for i, angle in enumerate(angles):
        joint_name = f'joint{i+1}'
        if angle < joint_limits[joint_name][0] or angle > joint_limits[joint_name][1]:
            raise ValueError(f"Joint {i+1} angle {angle} exceeds limits {joint_limits[joint_name]}")
    
    # Calculate forward kinematics
    tcp_pose = forward_kinematics_internal(angles)
    
    # Extract position and orientation
    position = tcp_pose[:3, 3]
    r11, r12, r13 = tcp_pose[0, :3]
    r21, r22, r23 = tcp_pose[1, :3]
    r31, r32, r33 = tcp_pose[2, :3]
    
    # Calculate RPY angles
    roll = np.arctan2(r32, r33)
    pitch = np.arctan2(-r31, np.sqrt(r32**2 + r33**2))
    yaw = np.arctan2(r21, r11)
    
    if not output_is_radian:
        roll, pitch, yaw = np.degrees([roll, pitch, yaw])
    
    return np.array([*position, roll, pitch, yaw])

def x6_get_ik(pose, input_is_radian=False, output_is_radian=False):
    """Calculate inverse kinematics for xArm 6.
    
    Args:
        pose: List of [x, y, z, roll, pitch, yaw] representing TCP pose
        input_is_radian: If True, input orientation is in radians; if False, in degrees
        output_is_radian: If True, output joint angles are in radians; if False, in degrees
    
    Returns:
        List of 6 joint angles
    """
    if len(pose) != 6:
        raise ValueError("Input pose must be a list of 6 values [x, y, z, roll, pitch, yaw]")
    
    pose = np.array(pose)
    if input_is_radian:
        pose[3:] = np.degrees(pose[3:])
    
    joint_angles = inverse_kinematics_internal(pose)
    
    if not output_is_radian:
        joint_angles = np.degrees(joint_angles)
    
    return joint_angles