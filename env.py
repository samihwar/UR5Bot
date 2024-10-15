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
        p.setGravity(0, 0, -10)
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
        visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.04, rgbaColor=[1, 0, 0, 1])#0.05
        collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_SPHERE, radius=0.04)#0.05
        ball_id = p.createMultiBody(baseMass=1,baseCollisionShapeIndex=collision_shape_id,baseVisualShapeIndex=visual_shape_id,basePosition=position)
        return ball_id
    
    # def pick_ball(self, ball_position):
    #     ball_x, ball_y, ball_z = ball_position
    #     action = [ball_x, ball_y, ball_z + 0.02, 0, 0, 0]  # go to the ball
    #     self.robot.move_ee(action, 'end')
    
    # def pick_ball(self, ball_position):
    #     ball_x, ball_y, ball_z = ball_position
    #     # define the action to move the end-effector to the ball's position
    #     action = [ball_x, ball_y, ball_z + 0.2, 0, 0, 0]  # move slightly above the ball 
    
    #     # print(f"target position: {action[:3]}")  # print the target position for debugging
    
    #     # move the end-effector to the target position
    #     self.robot.move_ee(action, 'end')
    #     time.sleep(5)
    #     action = [ball_x, ball_y, ball_z + 0.02, 0, 0, 0]
    #     self.robot.move_ee(action, 'end')
    #     # introduce a small delay to allow the movement to complete
    #     time.sleep(1)  # adjust the sleep time as necessary

    #     ee_pos = self.robot.get_joint_obs()['ee_pos']
    #     print(f"end-effector actual position: {ee_pos}")
    
    def move_to_position(self, target_position, control_method='end', steps=120):
        # mannualy end the simulation
        self.robot.move_ee(target_position, control_method)
        for _ in range(steps):
            p.stepSimulation()
            time.sleep(1 / 240.)  # Match the simulation time step

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

    def throw_ball(self, direction, speed=1.0, release_time=0.5):
        current_pos = self.robot.get_joint_obs()['ee_pos']
        target_pos = [
            current_pos[0] + direction[0] * speed,
            current_pos[1] + direction[1] * speed,
            current_pos[2] + direction[2] * speed,0,0,0
        ]
        self.move_to_position(target_pos)
        time.sleep(release_time)
        self.let_go_ball()



