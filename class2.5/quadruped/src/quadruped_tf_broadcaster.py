#!/usr/bin/env python3
import rospy
import numpy as np
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_matrix
import FK

class QuadrupedTFBroadcaster:
    def __init__(self):
        rospy.init_node('quadruped_tf_broadcaster', anonymous=True)

        # 創建 TF 廣播器
        self.tf_broadcaster = tf.TransformBroadcaster()

        # 訂閱 /joint_states
        self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        self.leg_names = ['FL', 'FR', 'RL', 'RR']
        self.leg_part_name = ['hip', 'thigh', 'calf', 'foot']

        # step 4 : 定義 body to hip 的 translation
        self.translation_dict = {
            'FL': np.array([??, ??, ??]),  # 左前腿
            'FR': np.array([??, ??, ??]),  # 右前腿
            'RL': np.array([??, ??, ??]),  # 左后腿
            'RR': np.array([??, ??, ??]),  # 右后腿
        }

    def get_leg_joint_positions(self, msg, leg):
        indices = {
            'FL': [0, 4, 8], 'FR': [1, 5, 9], 
            'RL': [2, 6, 10], 'RR': [3, 7, 11]
        }
        return np.array([msg.position[i] for i in indices[leg]])

    def get_dh_params(self, leg, current_q):
        # step 2 : 設計四隻腳的 DH table
        """ 根據不同的腿返回對應的 DH 參數表 """
        dh_params_dict = {
            'FL': [(??, ??, ??, ??), (??, ??, ??, ??), (??, ??, ??, ??)],
            'FR': [(??, ??, ??, ??), (??, ??, ??, ??), (??, ??, ??, ??)],
            'RL': [(??, ??, ??, ??), (??, ??, ??, ??), (??, ??, ??, ??)],
            'RR': [(??, ??, ??, ??), (??, ??, ??, ??), (??, ??, ??, ??)],
        }
        return dh_params_dict[leg]

    def joint_state_callback(self, msg):
        self.broadcast_transforms(msg)

    def get_body_to_hip_translation_matrix(self, leg):
        # step 5 : 定義 body to hip 的 roatation
        theta = np.radians(??)
        rotation_matrix = np.array([
            [??, ??, ??, 0],
            [??, ??, ??, 0],
            [??, ??, ??, 0],
            [0, 0, 0, 1]
        ])

        translation_matrix = rotation_matrix.copy()
        translation_matrix[:3, 3] = self.translation_dict[leg]

        return translation_matrix

    def broadcast_transforms(self, msg):
        time = rospy.Time.now()

        for leg in self.leg_names:
            current_q = self.get_leg_joint_positions(msg, leg)
            dh_params = self.get_dh_params(leg, current_q)

            # trunk to hip
            parent_frame = "trunk"
            child_frame = f"{leg}_{self.leg_part_name[0]}"
            transform_matrix = self.get_body_to_hip_translation_matrix(leg)
            self.publish_tf(transform_matrix, time, parent_frame, child_frame)

            # hip to thigh
            parent_frame = f"{leg}_{self.leg_part_name[0]}"
            child_frame = f"{leg}_{self.leg_part_name[1]}"
            if leg in ["FL", "RL"]:
                transform_matrix = FK.forward_kinematics_translation_matrix([dh_params[0]])
                transform_matrix[1, 3] = 0.062
            else:
                transform_matrix = FK.forward_kinematics_translation_matrix([dh_params[0]])
                transform_matrix[1, 3] = -0.062
            self.publish_tf(transform_matrix, time, parent_frame, child_frame)

            # thigh to calf
            parent_frame = f"{leg}_{self.leg_part_name[1]}"
            child_frame = f"{leg}_{self.leg_part_name[2]}"
            transform_matrix = FK.forward_kinematics_translation_matrix([dh_params[1]])
            self.publish_tf(transform_matrix, time, parent_frame, child_frame)

            # calf to foot
            parent_frame = f"{leg}_{self.leg_part_name[2]}"
            child_frame = f"{leg}_{self.leg_part_name[3]}"
            transform_matrix = FK.forward_kinematics_translation_matrix([dh_params[2]])
            self.publish_tf(transform_matrix, time, parent_frame, child_frame)

    def publish_tf(self, transform_matrix, time, parent_frame, child_frame):
        """
        將轉移矩陣轉換為 TF 並發布
        """
        translation = transform_matrix[:3, 3]

        # 提取旋轉矩陣並轉換為四元數
        quaternion = quaternion_from_matrix(transform_matrix)

        # 發布 TF
        self.tf_broadcaster.sendTransform(
            translation,
            quaternion,
            time,
            child_frame,
            parent_frame
        )

if __name__ == '__main__':
    node = QuadrupedTFBroadcaster()
    rospy.spin()
