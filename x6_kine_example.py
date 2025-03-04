import numpy as np
from x6_kinematics import x6_get_fk, x6_get_ik

# Example 1: Forward Kinematics
# Input joint angles in degrees: [-6.4, 28, -37.3, -0.5, 7.1, -69]
print("Example 1: Forward Kinematics")
print("Input joint angles (degrees):")
joint_angles = np.array([-6.4, 28, -37.3, -0.5, 7.1, -69])
print(f"[{', '.join([f'{angle:.6f}' for angle in joint_angles])}]")

# Calculate TCP pose using forward kinematics
tcp_pose = x6_get_fk(joint_angles, input_is_radian=False, output_is_radian=False)
print("\nOutput TCP pose [x, y, z, roll, pitch, yaw] (mm, degrees):")
print(f"[{', '.join([f'{value:.6f}' for value in tcp_pose])}]")

# Example 2: Inverse Kinematics
# Input TCP pose [x, y, z, roll, pitch, yaw]: [251.2, 0, 112, 180, 0, 0]
print("\nExample 2: Inverse Kinematics")
print("Input TCP pose [x, y, z, roll, pitch, yaw] (mm, degrees):")
tcp_pose = np.array([251.2, 0, 112, 180, 0, 0])
print(f"[{', '.join([f'{value:.6f}' for value in tcp_pose])}]")

# Calculate joint angles using inverse kinematics
joint_angles = x6_get_ik(tcp_pose, input_is_radian=False, output_is_radian=False)
print("\nOutput joint angles (degrees):")
print(f"[{', '.join([f'{angle:.6f}' for angle in joint_angles])}]")