# Robot Kinematics - UFACTORY xArm 6

This repository contains Python implementations of forward and inverse kinematics calculations for the UFACTORY xArm 6 robotic arm. It provides accurate and efficient functions for computing end-effector positions and joint angles.

## Compatibility

- Robot: UFACTORY xArm 6
- Python Version: 3.8 - 3.13

## Installation

1. Clone the repository:
```bash
git clone https://github.com/DanielWang123321/robot_kinematics.git
cd robot_kinematics
```

2. Install dependencies:
```bash
pip install numpy scipy pyyaml
```

## API Documentation

### Forward Kinematics

```python
x6_get_fk(angles, input_is_radian=False, output_is_radian=False)
```

Calculates the forward kinematics for xArm 6.

**Parameters:**
- `angles`: List of 6 joint angles
- `input_is_radian`: If True, input angles are in radians; if False, in degrees
- `output_is_radian`: If True, output orientation is in radians; if False, in degrees

**Returns:**
- List of [x, y, z, roll, pitch, yaw] representing TCP pose

### Inverse Kinematics

```python
x6_get_ik(pose, input_is_radian=False, output_is_radian=False)
```

Calculates the inverse kinematics for xArm 6.

**Parameters:**
- `pose`: List of [x, y, z, roll, pitch, yaw] representing TCP pose
- `input_is_radian`: If True, input orientation is in radians; if False, in degrees
- `output_is_radian`: If True, output joint angles are in radians; if False, in degrees

**Returns:**
- List of 6 joint angles

## Usage Example

The repository includes an example script (`x6_kine_example.py`) demonstrating both forward and inverse kinematics calculations:

```python
from x6_kinematics import x6_get_fk, x6_get_ik
import numpy as np

# Example 1: Forward Kinematics
joint_angles = [-6.4, 28, -37.3, -0.5, 7.1, -69]  # degrees
tcp_pose = x6_get_fk(joint_angles, input_is_radian=False, output_is_radian=False)
print("TCP pose [x, y, z, roll, pitch, yaw]:", tcp_pose)

# Example 2: Inverse Kinematics
tcp_pose = [251.2, 0, 112, 180, 0, 0]  # [mm, mm, mm, deg, deg, deg]
joint_angles = x6_get_ik(tcp_pose, input_is_radian=False, output_is_radian=False)
print("Joint angles:", joint_angles)
```

To run the example:
```bash
python x6_kine_example.py
```

## File Structure

- `x6_kine_para.yaml`: Robot parameters including MDH parameters and joint constraints
- `x6_kinematics.py`: Core kinematics calculations (forward/inverse kinematics)
- `x6_kine_example.py`: Usage examples