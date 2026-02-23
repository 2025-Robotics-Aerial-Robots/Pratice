#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import numpy as np
import FK
import IK_jacobian

class QuadrupedIK:
    def __init__(self):
        rospy.init_node('quadruped_ik', anonymous=True)

        # 訂閱 /joint_states
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        # 發布 /joint_command
        self.publisher = rospy.Publisher('/joint_command', JointState, queue_size=10)

        self.leg_names = ['FL', 'FR', 'RL', 'RR']
        self.joint_names = ['FL_hip_joint', 'FR_hip_joint', 'RL_hip_joint', 'RR_hip_joint',
                            'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint',
                            'FL_calf_joint', 'FR_calf_joint', 'RL_calf_joint', 'RR_calf_joint']
        
        # 運動參數
        self.A = 0.1  # 最大高度變化量（振幅）
        self.T = 5    # 週期（機器狗完整伏地挺身一次的時間）
        self.N = 50   # 離散點數量（越大越平滑）
        self.current_step = 0  # 當前的時間步
        self.initialized = False  # 確保只讀取初始狀態一次

        self.initial_foot_positions = {}  # 儲存初始腳的位置
        self.trajectory = []  # 預先規劃好的軌跡點

        # 100ms 更新一次
        rospy.Timer(rospy.Duration(0.1), self.update_target_positions)

    def get_leg_joint_positions(self, msg, leg):
        indices = {
            'FL': [0, 4, 8], 'FR': [1, 5, 9], 
            'RL': [2, 6, 10], 'RR': [3, 7, 11]
        }
        return np.array([msg.position[i] for i in indices[leg]])

    def get_dh_params(self, leg, current_q):
        """ 根據不同的腿返回對應的 DH 參數表 """
        # step 1 : 設計四隻腳的 DH table
        dh_params_dict = {
            'FL': [(??, ??, ??, ??), (??, ??, ??, ??), (??, ??, ??, ??)],
            'FR': [(??, ??, ??, ??), (??, ??, ??, ??), (??, ??, ??, ??)],
            'RL': [(??, ??, ??, ??), (??, ??, ??, ??), (??, ??, ??, ??)],
            'RR': [(??, ??, ??, ??), (??, ??, ??, ??), (??, ??, ??, ??)],
        }
        return dh_params_dict[leg]

    def initialize_foot_positions(self, msg):
        """ 讀取初始腳的位置 """
        for leg in self.leg_names:
            current_q = self.get_leg_joint_positions(msg, leg)
            dh_params = self.get_dh_params(leg, current_q)
            self.initial_foot_positions[leg] = FK.forward_kinematics(dh_params)

        # 計算平均高度
        x_avg = np.mean([pos[0] for pos in self.initial_foot_positions.values()])
        rospy.loginfo(f"Initialized foot positions. Avg X-height: {x_avg}")

    def plan_trajectory(self):
        """ 根據初始腳位置規劃完整軌跡 """
        self.trajectory = []
        for i in range(self.N):
            t = (i / self.N) * self.T  # 計算對應的時間
            x_offset = self.A * np.sin(2 * np.pi * t / self.T)  # sin 波軌跡

            step_positions = {
                leg: self.initial_foot_positions[leg] + np.array([x_offset, 0, 0]) for leg in self.leg_names
            }
            self.trajectory.append(step_positions)

        rospy.loginfo("Trajectory planned.")

    def update_target_positions(self, event):
        """ 更新當前的目標點，讓機器狗追隨軌跡 """
        if not self.trajectory:
            return

        self.current_step = (self.current_step + 1) % self.N  # 更新到下一個時間點
        rospy.loginfo(f"Moving to trajectory step: {self.current_step}")

    def joint_state_callback(self, msg):
        """ 讀取當前關節狀態，計算新的 IK 解，並發送指令 """
        joint_positions = [0] * 12  # 初始化 12 個關節角度

        if not self.initialized:
            self.initialize_foot_positions(msg)  # 讀取初始腳位置
            self.plan_trajectory()  # 規劃完整軌跡
            self.initialized = True  # 確保只執行一次

        for leg in self.leg_names:
            current_q = self.get_leg_joint_positions(msg, leg)
            target_pos = self.trajectory[self.current_step][leg]  # 取當前時間步的目標點
            new_q = self.inverse_kinematics(leg, current_q, target_pos)

            # 確保按照 joint_names 的正確索引順序填充
            leg_indices = {'FL': 0, 'FR': 1, 'RL': 2, 'RR': 3}
            idx = leg_indices[leg]

            joint_positions[idx]      = new_q[0]  # Hip
            joint_positions[idx + 4]  = new_q[1]  # Thigh
            joint_positions[idx + 8]  = new_q[2]  # Calf

        self.send_joint_commands(joint_positions)

    def inverse_kinematics(self, leg, current_q, target_pos):
        max_iterations = 5
        tolerance = 0.001

        # step 3 : 計算 Jacobian 使用數值解 解出IK
        for _ in range(max_iterations):
            dh_params = self.get_dh_params(leg, current_q)
            current_pos = FK.forward_kinematics(dh_params)
            error = target_pos - current_pos

            if np.linalg.norm(error) < tolerance:
                print("convergence")
                break

            jacobian = IK_jacobian.compute_jacobian(dh_params)
            delta_theta = ??? @ ???
            current_q += ???

        return current_q

    def send_joint_commands(self, joint_positions):
        msg = JointState()
        msg.name = self.joint_names  # 名稱順序不變
        msg.position = joint_positions
        print("publish")
        self.publisher.publish(msg)

if __name__ == '__main__':
    QuadrupedIK()
    rospy.spin()
