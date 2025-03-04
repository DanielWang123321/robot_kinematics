# robot_kinematics

A Python-based kinematics library for the xArm 6 robotic arm, implementing forward kinematics algorithms. This project has been validated on the xArm 6, aiming to provide an easy-to-use and extensible kinematics solution.

## Features

- **Forward Kinematics**: Compute the end-effector pose from given joint angles.
- Verified on the xArm 6 robotic arm.
- Clean, modular code for easy modification and expansion.

## Requirements

- Python 3.x
- NumPy library

## Installation

1. Clone the repository:
    ```bash
    git clone https://github.com/DanielWang123321/robot_kinematics.git
    ```
2. Change into the project directory:
    ```bash
    cd robot_kinematics
    ```
3. Install the dependencies:
    ```bash
    pip install numpy
    ```

## Usage

### Forward Kinematics

Use the functions in `x6_fk.py` to compute the end-effector pose from the joint angles.

Use the functions in `x6_ik.py` to compute the joint angles of target TCP pose.