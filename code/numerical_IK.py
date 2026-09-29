import sys
import os

import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from helpers.numeric_inv_kin_helpers import Numeric_IK_helper

# End of imports

def numeric_inv_kin(p, wrist, phi_prev):
    """
    INPUTS:
        p: desired end-effector position vector expressed in base frame {0}
        wrist: desired wrist rotation angle gamma in radians
        phi_prev: current/last joint positions

    OUTPUTS:
        phi_new : inverse kinematics solution
    """

    # Initilization
    L_1 = 0.1400
    L_2 = 0.3500
    L_3 = 0.0500
    L_4 = 0.2500
    L_5 = 0.1500
    BETA = np.arctan(L_3/L_2)

    LAMBDA_1 = L_1
    LAMBDA_2 = np.sqrt(L_2**2 + L_3**2)
    LAMBDA_3 = L_4 + L_5

    num_ik_helper = Numeric_IK_helper()

    #* Use this to get the inverse of the 4x4 analytical Jacobian:
    #J = num_ik_helper.Jacobian(phi)

    #* Use this to preform forward kinematics: p4 is the position of the end effector
    # p4, _ = num_ik_helper.forward_kinematics(phi)

    #* Use this to check the joint limits: True means valid position, False means joint limits exceeded
    # num_ik_helper.check_joint_limits(phi_new)

    # Solver Parameters
    success_thresh = 0.0001
    step_size_scaling = 0.01
    attempt_limit = 1000

    phi_new = phi_prev.copy()

    # == start student section ==
    #TODO: Implement numerical inverse kinematics

    # == end student section ==

    return phi_new


if __name__ == "__main__":
    print("Test your function here")
    p = np.array([0.0, -0.45, 0.5]) # desired end-effector position
    wrist = 0 # desired wrist position in radians
    phi_prev = np.radians(np.array([0.0, 0.0, 0.0, 0.0])) # current joint positions

    solution = numeric_inv_kin(p, wrist, phi_prev)
    print(solution)