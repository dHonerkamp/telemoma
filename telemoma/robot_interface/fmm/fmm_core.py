import os
import numpy as np
import rospy
from tf import transformations as T
from telemoma.utils.transformations import euler_to_quat, quat_to_euler, add_angles, quat_diff
from tracikpy import TracIKSolver
from rl_franka.fmm_interface_real import FmmInterfaceReal

class FMM:

    def __init__(self,
                    head_policy=None,
                    base_enabled=False,
                    torso_enabled=False,
                    arm_enabled=True):
        

        self.head_enabled = head_policy is not None
        self.base_enabled = base_enabled
        self.torso_enabled = torso_enabled
        self.arm_enabled = arm_enabled

        self.gripper_max = 0.08
        self.gripper_min = 0.0

        self.robot = FmmInterfaceReal()


        while not rospy.is_shutdown():
            try:

                dir_path = os.path.dirname(os.path.realpath(__file__))
                self.ik_solver = TracIKSolver(dir_path+"/../urdf/fmm_full.urdf", "base_link", "panda_hand",
                                              timeout=0.025, epsilon=5e-4, solve_type="Distance")

                break
            except:
                rospy.logerr_throttle('Failed to obtain resource: {}\nRetrying...')

        # Taken from fmm.yaml
        self.reset_pose = {
            "torso": 0.0,
            "arm" : np.array([-0.161, 0.448, 0.059, -1.074, 0.102, 1.420, 0.792]),
            "head": [0.0, 0.0]
        }

        self.lin_scale = 0.2
        self.ang_scale = 0.2            
        self.reference_pos = self.robot.get_base_pose_global()

    @property
    def eef_pose(self):
        return np.array(self.robot.get_franke_ee_pose_baseframe())

    @property
    def gripper_state(self):
 
        dist = self.robot.panda.gripper_pos

        return (dist - self.gripper_min) / (self.gripper_max - self.gripper_min)

    def get_delta_pose(self):
        # returns a 3d vector corresponding to change in position in x, y (2d) and change in angle in z (1d)
        current_pose = np.array(self.robot.get_base_pose_global())

        current_pos = current_pose[:3]
        reference_pos = self.reference_pos[:3]
        delta_pos = current_pos - reference_pos

        current_quat = current_pose[3:]
        reference_quat = self.reference_pos[3:]
        delta_quat = quat_diff(current_quat, reference_quat)
        delta_euler = quat_to_euler(delta_quat)

        return np.array([delta_pos[0], delta_pos[1], delta_euler[2]])

    def process_action(self, action):
        # convert deltas to absolute positions
        pos_delta, euler_delta, gripper = action[:3], action[3:6], action[6]
        cur_pose = self.eef_pose
        cur_pos, cur_euler = cur_pose[:3], quat_to_euler(cur_pose[3:])

        target_pos = cur_pos + pos_delta
        target_euler = add_angles(euler_delta, cur_euler)
        target_quat = euler_to_quat(target_euler)

        return target_pos, target_quat, gripper

    def step(self, action):

        if self.arm_enabled:
            pos, quat, gripper_act = self.process_action(action['left'])

            # trac-ik
            ee_matrix = T.quaternion_matrix(quat)
            ee_matrix = np.dot(T.translation_matrix(pos), ee_matrix)
            ik_solution = self.ik_solver.ik(ee_matrix, qinit=np.zeros(self.ik_solver.number_of_joints))

            if ik_solution is None:
                print('No IK solution for ', pos, quat, self.eef_pose)
            else:
                torso_pos, joint_positions = ik_solution[0], ik_solution[1:]
                self.robot.panda.async_move_joint_position(joint_positions, vel_scale=0.4)
                self.robot.pub_tower_q(torso_pos)

            if abs(gripper_act - self.gripper_state) > 0.2:
                print (gripper_act, self.gripper_state)
                self.robot.panda.async_set_gripper_position(gripper_act)

        if self.base_enabled:
            base_pose = action['base']
            if base_pose is None:
                self.robot.pub_base_qd([0., 0., 0.])
            else:
                lin_cmd = np.zeros(3)
                lin_cmd[:2] = base_pose[:2]
                lin_cmd = self.lin_scale * lin_cmd

                ang_cmd = np.zeros(3)
                if abs(base_pose[2]) > 0.25:
                    ang_cmd[2] = base_pose[2]
                    ang_cmd = self.ang_scale * ang_cmd

                self.robot.pub_base_qd([lin_cmd[0], lin_cmd[1], ang_cmd[2]])

    def reset(self, reset_arms=True):
        if reset_arms:
            if self.reset_pose is not None:
                self.robot.panda.async_move_joint_position(self.reset_pose["arm"], vel_scale=0.4)
                self.robot.pub_tower_q(self.reset_pose["torso"])

        # Skipping head reset
        # if self.reset_pose is not None:
        #     self.whole_body.move_to_joint_positions(goals=self.reset_pose["head"])

        input('Reset complete. Press ENTER to continue')

