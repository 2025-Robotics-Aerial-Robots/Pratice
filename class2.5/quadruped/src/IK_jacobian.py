import numpy as np
import FK

def compute_jacobian(dh_params):
    T_total = np.eye(4)
    positions = []
    rotations = []
    
    for params in dh_params:
        T = FK.dh_transform(*params)
        T_total = T_total @ T
        positions.append(T_total[:3, 3])
        rotations.append(T_total[:3, :3])

    # step 2 : Compute Jacobian matrix J
    
    p1, p2, p3 = positions
    
    R1, R2, _ = rotations

    z0 = np.array([0, 0, 1])
    
    Jv1 = np.cross(??, ??)
    Jv2 = np.cross(?? @ ??, (?? - ??))
    Jv3 = np.cross(?? @ ??, (?? - ??))
    
    J = np.column_stack([Jv1, Jv2, Jv3])

    return J