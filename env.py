import time
import math
import random

import numpy as np
import pybullet as p
import pybullet_data
from robot import RobotBase

from utilities import Models, Camera
from collections import namedtuple
from attrdict import AttrDict
from tqdm import tqdm


class FailToReachTargetError(RuntimeError):
    pass


class ClutteredPushGrasp:

    SIMULATION_STEP_DELAY = 1 / 240.

    def __init__(self, robot, models: Models, camera=None, vis=False) -> None:
        self.robot = robot
        self.vis = vis
        if self.vis:
            self.p_bar = tqdm(ncols=0, disable=False)
        self.camera = camera

        # define environment
        self.physicsClient = p.connect(p.GUI if self.vis else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.planeID = p.loadURDF("plane.urdf")

        self.robot.load()
        self.robot.step_simulation = self.step_simulation

        # custom sliders to tune parameters (name of the parameter,range,initial value)
        self.xin = p.addUserDebugParameter("x", -0.224, 0.224, 0)
        self.yin = p.addUserDebugParameter("y", -0.224, 0.224, 0)
        self.zin = p.addUserDebugParameter("z", 0, 1., 0.5)
        self.rollId = p.addUserDebugParameter("roll", -3.14, 3.14, 0)
        self.pitchId = p.addUserDebugParameter("pitch", -3.14, 3.14, np.pi/2)
        self.yawId = p.addUserDebugParameter("yaw", -np.pi/2, np.pi/2, np.pi/2)
        self.gripper_opening_length_control = p.addUserDebugParameter("gripper_opening_length", 0, 0.085, 0.04)

        joint_obs = self.robot.get_joint_obs()
        print(joint_obs.keys())

        # self.boxID = p.loadURDF("./urdf/skew-box-button.urdf",
        #                         [0.0, 0.0, 0.0],
        #                         # p.getQuaternionFromEuler([0, 1.5706453, 0]),
        #                         p.getQuaternionFromEuler([0, 0, 0]),
        #                         useFixedBase=True,
        #                         flags=p.URDF_MERGE_FIXED_LINKS | p.URDF_USE_SELF_COLLISION)

        # # For calculating the reward
        # self.box_opened = False
        # self.btn_pressed = False
        # self.box_closed = False

    def step_simulation(self):
        """
        Hook p.stepSimulation()
        """
        p.stepSimulation()
        if self.vis:
            time.sleep(self.SIMULATION_STEP_DELAY)
            self.p_bar.update(1)

    def read_debug_parameter(self):
        # read the value of task parameter
        x = p.readUserDebugParameter(self.xin)
        y = p.readUserDebugParameter(self.yin)
        z = p.readUserDebugParameter(self.zin)
        roll = p.readUserDebugParameter(self.rollId)
        pitch = p.readUserDebugParameter(self.pitchId)
        yaw = p.readUserDebugParameter(self.yawId)
        gripper_opening_length = p.readUserDebugParameter(self.gripper_opening_length_control)

        return x, y, z, roll, pitch, yaw, gripper_opening_length

    # def step(self, action, control_method='joint'):
    #     """
    #     action: (x, y, z, roll, pitch, yaw, gripper_opening_length) for End Effector Position Control
    #             (a1, a2, a3, a4, a5, a6, a7, gripper_opening_length) for Joint Position Control
    #     control_method:  'end' for end effector position control
    #                      'joint' for joint position control
    #     """
    #     assert control_method in ('joint', 'end')
    #     self.robot.move_ee(action[:-1], control_method)
    #     self.robot.move_gripper(action[-1])
    #     for _ in range(120):  # Wait for a few steps
    #         self.step_simulation()

    #     reward = self.update_reward()
    #     done = True if reward == 1 else False
    #     info = dict(box_opened=self.box_opened, btn_pressed=self.btn_pressed, box_closed=self.box_closed)
    #     return self.get_observation(), reward, done, info

    # def update_reward(self):
    #     reward = 0
    #     if not self.box_opened:
    #         if p.getJointState(self.boxID, 1)[0] > 1.9:
    #             self.box_opened = True
    #             print('Box opened!')
    #     elif not self.btn_pressed:
    #         if p.getJointState(self.boxID, 0)[0] < - 0.02:
    #             self.btn_pressed = True
    #             print('Btn pressed!')
    #     else:
    #         if p.getJointState(self.boxID, 1)[0] < 0.1:
    #             print('Box closed!')
    #             self.box_closed = True
    #             reward = 1
    #     return reward

    def get_observation(self):
        obs = dict()
        if isinstance(self.camera, Camera):
            rgb, depth, seg = self.camera.shot()
            obs.update(dict(rgb=rgb, depth=depth, seg=seg))
        else:
            assert self.camera is None
        obs.update(self.robot.get_joint_obs())

        return obs

    # def reset_box(self):
    #     p.setJointMotorControl2(self.boxID, 0, p.POSITION_CONTROL, force=1)
    #     p.setJointMotorControl2(self.boxID, 1, p.VELOCITY_CONTROL, force=0)

    def reset(self):
        self.robot.reset()
        # self.reset_box()
        return self.get_observation()

    def close(self):
        p.disconnect(self.physicsClient)
        
    # i wanna try to add a ball and make UR5 pick it up

    def load_ball(self, position): # creating the ball itself at a position
        visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.04, rgbaColor=[1, 0, 0, 1])# the size was 0.05
        collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_SPHERE, radius=0.04)# the size was 0.05
        ball_id = p.createMultiBody(baseMass=1,baseCollisionShapeIndex=collision_shape_id,baseVisualShapeIndex=visual_shape_id,basePosition=position)
        return ball_id
    
    def move_to_position(self, target_position, control_method='end', steps=120):
        self.robot.move_ee(target_position, control_method)
        for _ in range(steps):
            p.stepSimulation()
            time.sleep(1 / 240.)

    def hug_ball(self, ball_position):
        pitch=math.pi / 2   # look down
        ball_x, ball_y, ball_z = ball_position
    
        action = [ball_x, ball_y, ball_z + 0.4, 0, pitch, 0]  # Move slightly above the ball
        self.move_to_position(action)
        time.sleep(2)
        ee_pos = self.robot.get_joint_obs()['ee_pos']
        print(f"end-effector actual position: {ee_pos}")
    
        action = [ball_x, ball_y, ball_z + 0.11, 0, pitch, 0]  # Adjust the z position to be closer to the ball 
        self.move_to_position(action)
        time.sleep(1)
        ee_pos = self.robot.get_joint_obs()['ee_pos']
        print(f"end-effector actual position: {ee_pos}")

    def grasp_ball(self):
        self.robot.close_gripper()
        for _ in range(60):  # Wait for the gripper to close
            p.stepSimulation()
            time.sleep(1 / 240.)

    def get_the_ball(self, ball_position):
        self.hug_ball(ball_position)
        self.grasp_ball()

    def let_go_ball(self):
        self.robot.open_gripper()
        for _ in range(60):  # Wait for the gripper to close
            p.stepSimulation()
            time.sleep(1 / 240.)
            
    def lift_ball(self,lift_height=0.1):
        current_pos = self.robot.get_joint_obs()['ee_pos']
        pitch=math.pi / 2
        action = [current_pos[0], current_pos[1], current_pos[2] + lift_height, 0, pitch, 0]  # Lift the ball by lift_height units
        self.move_to_position(action)
        time.sleep(1)
        ee_pos = self.robot.get_joint_obs()['ee_pos']
        print(f"end-effector actual position: {ee_pos}")

    # def throw_ball(self, direction, speed=1.0, release_time=0.5):
    #     current_pos = self.robot.get_joint_obs()['ee_pos']
    #     target_pos = [
    #         current_pos[0] + direction[0] * speed,
    #         current_pos[1] + direction[1] * speed,
    #         current_pos[2] + direction[2] * speed,0,0,0
    #     ]
    #     self.move_to_position(target_pos)
    #     time.sleep(release_time)
    #     self.let_go_ball()

    def move_joint_2(self, target_angle):
        # get joint positions
        joint_obs = self.robot.get_joint_obs()
        current_joint_positions = joint_obs['positions']
        modified_joint_positions = current_joint_positions.copy()

        modified_joint_positions[2] = target_angle

        self.robot.set_joint_positions(modified_joint_positions)


    def throw_ball(self, direction, speed=1.0, release_time=0.5):
        start_position = [0.3, 0.5, 0.3, 0, 0, 0]
        self.move_to_position(start_position)
    
        # Move only Joint 2
        new_joint_2_angle = -0.1
        self.move_joint_2(new_joint_2_angle)
    
        time.sleep(release_time)
        self.let_go_ball()


    # def throw_ball(self, direction, speed=1.0, release_time=0.5):
    #     joint_obs = self.robot.get_joint_obs()
    
    #     # Print the entire observation structure to inspect the available keys
    #     print("Joint Observation Structure:", joint_obs)

    #     # Step 1: Move to the start position (same as before)
    #     start_position = [0.1, 0.1, 0.3, 0, 0, 0]  # This is the full EE position
    #     self.robot.move_ee(start_position, control_method='joint')

    #     # Step 2: Get the current joint states (assuming you have 6 joints)
    #     joint_obs = self.robot.get_joint_obs()
    #     current_joints = joint_obs['4']

    #     # Step 3: Modify the desired joint (e.g., joint 4 for wrist or elbow)
    #     modified_joints = current_joints.copy()  # Copy current joint positions
    #     wrist_joint_index = 4  # Example: Assume joint 4 is the wrist
    #     modified_joints[wrist_joint_index] += direction[0] * speed  # Adjust only the wrist

    #     # Step 4: Send the new joint configuration (move only the wrist)
    #     self.robot.set_joint_positions(modified_joints)  # Replace with the actual function to set joints
    #     time.sleep(release_time)
    #     self.let_go_ball()
    #     # self.robot.set_joint_positions(current_joints)


    # def throw_ball(self, direction, speed=1.0, release_time=0.5):
    #     gravity = -9.81  # Simulated gravity effect in z direction
    #     time_step = 0.02  # Time interval for smooth movement
    #     num_steps = int(release_time / time_step)  # Number of steps for throw
    
    #     current_pos = self.robot.get_joint_obs()['ee_pos']
    #     velocity = np.array(direction) * speed  # Initial velocity vector
    #     position = np.array(current_pos[:3])  # Only take x, y, z
    
    #     # Set a default orientation (roll, pitch, yaw) if not available
    #     roll, pitch, yaw = 0, 0, 0  # Default values
    
    #     print(f"Starting position: {position}")
    
    #     for step in range(num_steps):
    #         # Update position based on current velocity
    #         position += velocity * time_step
        
    #         # Apply gravity only in z direction
    #         velocity[2] += gravity * time_step
        
    #         # Debug: print position and velocity to make sure they are changing
    #         print(f"Step {step + 1}: Position: {position}, Velocity: {velocity}")
        
    #         # Move to the new position and use the default orientation
    #         self.move_to_position(position.tolist() + [roll, pitch, yaw])
        
    #         # Short sleep to simulate real-time movement
    #         time.sleep(time_step)
    
    #     # Let go of the ball at the end of the motion
    #     self.let_go_ball()




