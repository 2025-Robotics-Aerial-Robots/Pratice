import numpy as np

def dh_transform(theta, d, a, alpha):
    """
    計算 Denavit-Hartenberg 變換矩陣
    """
    # step 3 :　DH 轉移矩陣
    return np.array([
            [??, ??, ??, ??],
            [??, ??, ??, ??],
            [??, ??, ??, ??],
            [??, ??, ??, ??]
        ])

def forward_kinematics(dh_params):
    """
    使用 DH 參數計算末端點位置
    """
    T_total = np.eye(4)
    for params in dh_params:
        T = dh_transform(*params)
        T_total = T_total @ T
    return T_total[:3,3]

def forward_kinematics_translation_matrix(dh_params):
    """
    使用 DH 參數計算末端點位置
    """
    T_total = np.eye(4)
    for params in dh_params:
        T = dh_transform(*params)
        T_total = T_total @ T
    return T_total