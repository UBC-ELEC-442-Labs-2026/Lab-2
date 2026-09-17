import numpy as np
import constants
import importlib.util
import sys
import os

# Import the QArm interface class
interface_directory = os.path.dirname(constants.path_to_interface)

if interface_directory not in sys.path:
    sys.path.append(interface_directory)

from QArm_functions import QArm_Lab_interface #type: ignore

# End of imports

def numeric_inv_kin(p, wrist, phi_prev):
    """
    INPUTS:
        p: desired end-effector position vector expressed in base frame {0}
        wrist: desired wrist rotation angle gamma in degrees
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

    QArm_interface = QArm_Lab_interface()

    #* Use this to get the inverse of the 4x4 analytical Jacobian:
    #J_inv = QArm_interface.Inv_Jacobian(phi)

    #* Use this to preform forward kinematics: p4 is the position of the end effector
    # p4, _ = QArm_interface.forward_kinematics(phi)

    #* Use this to check the joint limits: True means okay position
    # QArm_interface.check_joint_limits(phi_new)

    success_thresh = 0.0001
    step_size_scaling = 0.01
    attempt_limit = 1000

    # == start student section ==
    #TODO: Implement numerical inverse kinematics

    phi_new = phi_prev.copy()

    # == end student section ==

    return phi_new


if __name__ == "__main__":
    print("Test your function here")
    p = np.array([0.0, -0.45, 0.5]) # desired end-effector position
    wrist = 0 # desired wrist position in radians
    phi_prev = np.radians(np.array([0.0, 0.0, 0.0, 0.0])) # current joint positions

    solution = numeric_inv_kin(p, wrist, phi_prev)
    print(solution)