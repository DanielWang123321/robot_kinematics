import numpy as np
import yaml
from scipy.optimize import least_squares

# Load kinematics parameters from YAML file
def load_kinematics_params(yaml_file):
    with open(yaml_file, 'r') as file:
        params = yaml.safe_load(file)
    return params

# Load parameters from YAML file
kin_params = load_kinematics_params('x6_kin.yaml')

# Extract joint limits
joint_limits = kin_params['joint_limits']

# Calculate Denavit-Hartenberg transformation matrix
def dh_transform(alpha, a, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -d * np.sin(alpha)],
        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), d * np.cos(alpha)],
        [0, 0, 0, 1]
    ])

# Extract MDH parameters
dh_yaml = kin_params['dh_params']
dh_params = [
    dh_yaml['joint1'],  # Joint 1
    dh_yaml['joint2'],  # Joint 2
    dh_yaml['joint3'],  # Joint 3
    dh_yaml['joint4'],  # Joint 4
    dh_yaml['joint5'],  # Joint 5
    dh_yaml['joint6']   # Joint 6
]

# Tool Coordinate System (TCP) Transformation
T_tool = np.array(kin_params['tool']['transform'])  # Load from YAML file

# Forward Kinematics function (needed for IK calculations)
def forward_kinematics(joint_angles):
    T = np.eye(4)
    for i, (alpha, a, d, theta_offset) in enumerate(dh_params):
        theta = joint_angles[i] + theta_offset  # Add MDH offset
        T = T @ dh_transform(alpha, a, d, theta)  # Cascade transformation matrices
    T = T @ T_tool  # Multiply by tool coordinate transformation
    return T

# Calculate Jacobian matrix numerically
def calculate_jacobian(joint_angles, delta=0.0001):
    # Initialize Jacobian matrix (6×6 for 6-DOF robot)
    J = np.zeros((6, 6))
    
    # Get current end-effector pose
    current_pose = forward_kinematics(joint_angles)
    current_position = current_pose[:3, 3]
    
    # Extract rotation matrix and convert to RPY angles
    r11, r12, r13 = current_pose[0, :3]
    r21, r22, r23 = current_pose[1, :3]
    r31, r32, r33 = current_pose[2, :3]
    
    current_roll = np.arctan2(r32, r33)
    current_pitch = np.arctan2(-r31, np.sqrt(r32**2 + r33**2))
    current_yaw = np.arctan2(r21, r11)
    current_orientation = np.array([current_roll, current_pitch, current_yaw])
    
    # Calculate Jacobian columns using finite differences
    for i in range(6):
        # Perturb joint angle slightly
        perturbed_angles = joint_angles.copy()
        perturbed_angles[i] += delta
        
        # Calculate new pose
        perturbed_pose = forward_kinematics(perturbed_angles)
        perturbed_position = perturbed_pose[:3, 3]
        
        # Extract rotation matrix and convert to RPY angles
        r11, r12, r13 = perturbed_pose[0, :3]
        r21, r22, r23 = perturbed_pose[1, :3]
        r31, r32, r33 = perturbed_pose[2, :3]
        
        perturbed_roll = np.arctan2(r32, r33)
        perturbed_pitch = np.arctan2(-r31, np.sqrt(r32**2 + r33**2))
        perturbed_yaw = np.arctan2(r21, r11)
        perturbed_orientation = np.array([perturbed_roll, perturbed_pitch, perturbed_yaw])
        
        # Calculate position and orientation differences
        position_diff = (perturbed_position - current_position) / delta
        orientation_diff = (perturbed_orientation - current_orientation) / delta
        
        # Fill Jacobian column
        J[:3, i] = position_diff
        J[3:, i] = orientation_diff
    
    return J

# Error function for optimization
def pose_error(joint_angles, target_pose):
    # Calculate current pose
    current_pose = forward_kinematics(joint_angles)
    
    # Extract position
    current_position = current_pose[:3, 3]
    target_position = target_pose[:3]
    
    # Extract rotation matrix and convert to RPY angles
    r11, r12, r13 = current_pose[0, :3]
    r21, r22, r23 = current_pose[1, :3]
    r31, r32, r33 = current_pose[2, :3]
    
    current_roll = np.degrees(np.arctan2(r32, r33))
    current_pitch = np.degrees(np.arctan2(-r31, np.sqrt(r32**2 + r33**2)))
    current_yaw = np.degrees(np.arctan2(r21, r11))
    
    current_orientation = np.array([current_roll, current_pitch, current_yaw])
    target_orientation = target_pose[3:]
    
    # Combine position and orientation errors
    position_error = current_position - target_position
    orientation_error = current_orientation - target_orientation
    
    # Normalize orientation error (handle angle wrapping)
    for i in range(3):
        while orientation_error[i] > 180:
            orientation_error[i] -= 360
        while orientation_error[i] < -180:
            orientation_error[i] += 360
    
    # Calculate joint angle minimization term - increase weight significantly
    joint_minimization = np.abs(joint_angles)  # Sum of absolute joint angles
    joint_weight = 0.1  # Further increased weight factor for joint minimization
    
    # Return combined error with joint minimization
    return np.concatenate([
        position_error,
        orientation_error / 10,  # Scale orientation errors
        joint_minimization * joint_weight  # Add joint minimization term with higher weight
    ])

# Check if joint angles are within limits
def check_joint_limits(joint_angles):
    for i, angle in enumerate(joint_angles):
        joint_name = f'joint{i+1}'
        lower_limit, upper_limit = joint_limits[joint_name]
        
        if angle < lower_limit:
            joint_angles[i] = lower_limit
        elif angle > upper_limit:
            joint_angles[i] = upper_limit
    
    return joint_angles

# Inverse Kinematics using Jacobian-based method with multiple initial guesses
def inverse_kinematics(target_pose, initial_guess=None, max_iterations=1000, tolerance=1e-3):
    # If no initial guess provided, use zeros
    if initial_guess is None:
        initial_guess = np.zeros(6)
    
    # Define multiple initial guesses to try
    initial_guesses = [
        initial_guess,  # Original initial guess
        np.zeros(6),    # Zero configuration
        np.array([0, -np.pi/4, -np.pi/4, 0, 0, 0]),  # Common robot pose
        np.array([np.pi/2, 0, 0, 0, np.pi/2, 0]),    # Alternative configuration
        np.array([-np.pi/2, 0, 0, 0, -np.pi/2, 0])   # Another alternative
    ]
    
    # Convert target orientation from degrees to radians for internal calculations
    target_pose_internal = target_pose.copy()
    target_pose_internal[3:] = np.radians(target_pose_internal[3:])
    
    best_solution = None
    best_error = float('inf')
    best_joint_sum = float('inf')
    
    # Try each initial guess
    for guess in initial_guesses:
        # Define optimization function
        def objective(joint_angles):
            return pose_error(joint_angles, target_pose)
        
        # Use least squares optimization
        result = least_squares(
            objective,
            guess,
            bounds=([joint_limits[f'joint{i+1}'][0] for i in range(6)], 
                    [joint_limits[f'joint{i+1}'][1] for i in range(6)]),
            method='trf',
            ftol=tolerance,
            max_nfev=max_iterations
        )
        
        # Get optimized joint angles
        joint_angles = result.x
        
        # Calculate final error
        final_error = pose_error(joint_angles, target_pose)
        position_error = np.linalg.norm(final_error[:3])
        orientation_error = np.linalg.norm(final_error[3:6])  # Only consider the orientation part
        total_error = position_error + orientation_error
        
        # Calculate total joint movement (absolute sum)
        joint_sum = np.sum(np.abs(joint_angles))
        
        # Check if this solution is better
        # First prioritize solutions with acceptable accuracy
        if total_error < 1.0:  # Threshold for acceptable accuracy
            # Then among accurate solutions, choose the one with minimum joint movement
            if joint_sum < best_joint_sum:
                best_solution = joint_angles
                best_error = total_error
                best_joint_sum = joint_sum
        # If we don't have any good solutions yet, take the most accurate one
        elif best_solution is None or total_error < best_error:
            best_solution = joint_angles
            best_error = total_error
            best_joint_sum = joint_sum
    
    # Use the best solution found
    joint_angles = best_solution
    
    # Calculate final error for reporting
    final_error = pose_error(joint_angles, target_pose)
    position_error = np.linalg.norm(final_error[:3])
    orientation_error = np.linalg.norm(final_error[3:6])  # Only consider the orientation part
    
    print(f"Position error: {position_error:.4f} mm")
    print(f"Orientation error: {orientation_error:.4f} degrees")
    
    return joint_angles

# Convert TCP pose [x, y, z, roll, pitch, yaw] to joint angles
def tcp_to_joint_angles(tcp_pose, initial_guess=None):
    # Convert TCP pose to numpy array
    tcp_pose = np.array(tcp_pose)
    
    # Call inverse kinematics solver
    joint_angles = inverse_kinematics(tcp_pose, initial_guess)
    
    # Convert to degrees for output
    joint_angles_deg = np.degrees(joint_angles)
    
    return joint_angles_deg

# Test with the given TCP position [220,0,112,180,0,0] (mm and degrees)
if __name__ == "__main__":
    # Target TCP pose [x, y, z, roll, pitch, yaw]
    target_tcp = np.array([376.4,-210.9,112,163.5,0,-75.4])
    
    print("Target TCP pose:")
    print(f"Position (mm): [{target_tcp[0]}, {target_tcp[1]}, {target_tcp[2]}]")
    print(f"Orientation (°): [Roll: {target_tcp[3]}, Pitch: {target_tcp[4]}, Yaw: {target_tcp[5]}]")
    
    # Initial guess (can be adjusted if needed)
    initial_guess = np.radians([0, 0, 0, 0, 0, 0])
    
    # Calculate inverse kinematics
    joint_angles_deg = tcp_to_joint_angles(target_tcp, initial_guess)
    
    print("\nCalculated joint angles (degrees):")
    for i, angle in enumerate(joint_angles_deg):
        print(f"Joint {i+1}: {angle:.4f}°")
    
    # Verify solution with forward kinematics
    joint_angles_rad = np.radians(joint_angles_deg)
    tcp_pose = forward_kinematics(joint_angles_rad)
    
    # Extract TCP position (mm)
    tcp_position = tcp_pose[:3, 3]
    
    # Calculate TCP rotation (RPY angles)
    r11, r12, r13 = tcp_pose[0, :3]
    r21, r22, r23 = tcp_pose[1, :3]
    r31, r32, r33 = tcp_pose[2, :3]
    
    roll = np.degrees(np.arctan2(r32, r33))
    pitch = np.degrees(np.arctan2(-r31, np.sqrt(r32**2 + r33**2)))
    yaw = np.degrees(np.arctan2(r21, r11))
    
    print("\nVerification using forward kinematics:")
    print(f"TCP position (mm): {tcp_position}")
    print(f"TCP rotation (°): [Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}]")