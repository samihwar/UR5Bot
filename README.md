# UR5Bot
![image](https://github.com/user-attachments/assets/fa6adced-5bc4-4435-9cf4-3d7c2da94ea1)

This is a robotic simulation framework designed for precise robotic arm control using **torque control**. It utilizes **PyBullet** for physics simulation and allows fine-grained manipulation of robotic joints via user-defined sliders.

## Features
- **Torque Control**: Directly control joint torques for smooth and precise movements.
- **User Debug Sliders**: Adjust joint torques interactively using PyBullet's UI sliders.
- **Custom Joint Initialization**: Start with predefined joint configurations.
- **UR5 and Robotiq 2F-85 Support**: Configured for use with a UR5 robotic arm and a Robotiq 2F-85 gripper.

## Prerequisites
- Python 3
- PyBullet

## Installation

1. Clone the repository:
   ```sh
   git clone https://github.com/samihwar/UR5Bot.git
   cd UR5Bot
   ```

## Usage

### Running the Simulation
Run the main script to launch the PyBullet simulation:
   ```sh
   python main.py
   ```

### Controlling the Robot
The robot's joints can be controlled through **torque control** using sliders. Each joint has a corresponding slider to modify its torque.

## Code Overview

- `robot.py` - Defines the robotic arm, loads URDF, and manages joint properties.
- `simulation.py` - Handles PyBullet environment setup and physics simulation.
- `control.py` - Implements torque control for the robot.
- `main.py` - Entry point to start the simulation.

## Joint Initialization

The robot's joints start at predefined positions:
```python
starting_joints = {
    'shoulder_pan_joint': -1.57,
    'shoulder_lift_joint': -1.55,
    'elbow_joint': 1.37,
    'wrist_1_joint': -1.39,
    'wrist_2_joint': -1.57,
    'wrist_3_joint': 0.0,
    'finger_joint': 0.0,
    'left_inner_finger_joint': 0.0,
    'left_inner_knuckle_joint': 0.0,
    'right_outer_knuckle_joint': 0.0,
    'right_inner_finger_joint': 0.0,
    'right_inner_knuckle_joint': 0.44,
    'gripper_opening_length': 0.04
}
```

## Future Improvements
- Improved control stability and response time.
- Integration with external controllers.
- Throwing a ball in a precise way.

## License
This project is open-source and available under the MIT License.

## 🙏 Credits
This Project was done as a semester project for a BSc in Computer Science at the University of Haifa, under the supervision of Prof. Roi Poranne.

- **Samih Warwar** - Creator and Developer
- **Prof. Roi Poranne** - Supervisor and Mentor

## ☎ Contact

- **Github Profile**: [https://github.com/samihwar](https://github.com/samihwar)
- **📧 Email**: [samih.warwar@gmail.com](mailto:samih.warwar@gmail.com)
- **LinkedIn**: [https://www.linkedin.com/in/samih-warwar](https://www.linkedin.com/in/samih-warwar)

