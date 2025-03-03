import numpy as np

# Joint limits in radians
joint_limits = {
    "joint1": [-2 * np.pi, 2 * np.pi],
    "joint2": [-2.059488, 2.094395],
    "joint3": [-3.927231, 0.191986],
    "joint4": [-2 * np.pi, 2 * np.pi],
    "joint5": [-1.692969, np.pi],
    "joint6": [-2 * np.pi, 2 * np.pi],
}

# Calculate Denavit-Hartenberg transformation matrix
def dh_transform(alpha, a, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -d * np.sin(alpha)],
        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), d * np.cos(alpha)],
        [0, 0, 0, 1]
    ])

# xArm 6 MDH parameters
a2 = np.sqrt(284.5**2 + 53.5**2)  # 289.48866 mm
T2_offset = -np.arctan(284.5 / 53.5)  # -1.3849179 rad
T3_offset = -T2_offset  # 1.3849179 rad

# MDH 参数表：[alpha, a, d, theta_offset]
dh_params = [
    [0.000000, 0.000000, 267.000, 0],           # Joint 1
    [-np.pi/2, 0.000000, 0.000000, T2_offset],  # Joint 2
    [0.000000, a2, 0.000000, T3_offset],        # Joint 3
    [-np.pi/2, 77.500, 342.500, 0],             # Joint 4
    [np.pi/2, 0.000000, 0.000000, 0],           # Joint 5
    [-np.pi/2, 76.000, 97.000, 0]               # Joint 6
]

# 工具坐标系 (TCP) 变换
T_tool = np.eye(4)  # 若有额外的 TCP 偏移，可在此调整

# 计算正向运动学 (Forward Kinematics)
def forward_kinematics(joint_angles):
    T = np.eye(4)
    for i, (alpha, a, d, theta_offset) in enumerate(dh_params):
        theta = joint_angles[i] + theta_offset  # 加上 MDH 偏移
        T = T @ dh_transform(alpha, a, d, theta)  # 级联变换矩阵
    T = T @ T_tool  # 乘以工具坐标变换
    return T

# 关节零位姿态
# joint_angles = np.radians([0,0,0,0,0,0])
joint_angles = np.radians([ 10.0, 0,-30,0,0,0])


tcp_pose = forward_kinematics(joint_angles)

# 提取 TCP 位置 (mm)
tcp_position = tcp_pose[:3, 3]  # 末端位姿
tcp_position_mm = tcp_position * 1.0  # 单位转换 mm

# 计算 TCP 旋转 (RPY 角度)
r11, r12, r13 = tcp_pose[0, :3]
r21, r22, r23 = tcp_pose[1, :3]
r31, r32, r33 = tcp_pose[2, :3]

roll = np.degrees(np.arctan2(r32, r33))
pitch = np.degrees(np.arctan2(-r31, np.sqrt(r32**2 + r33**2)))
yaw = np.degrees(np.arctan2(r21, r11))

# 输出结果
print(f"TCP 位置 (mm): {tcp_position_mm}")
print(f"TCP 旋转 (°): [Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}]")
