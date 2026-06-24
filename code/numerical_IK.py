import numpy as np
import constants
import importlib.util
import sys

file_path = constants.path_to_interface
class_name = "QArm_Lab_interface"
module_name = "QArm_Lab_interface_module"
spec = importlib.util.spec_from_file_location(module_name, file_path)
module = importlib.util.module_from_spec(spec)
sys.modules[module_name] = module
spec.loader.exec_module(module)
QArm_Lab_interface = getattr(module, class_name)

def numeric_inv_kin(p, wrist, phi_prev):
    """
    INPUTS:
        p: end-effector position vector expressed in base frame {0}
        wrist: wrist rotation angle gamma

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

    QArmUtilities = QArm_Lab_interface.__bases__[0]

    # Use this to get the 4x4 analytical Jacobian:
    # J, _, _, _ = QArmUtilities.differential_kinematics(phi_prev)

    # == start student section ==
    #TODO: Implement numerical inverse kinematics

    phi_new = None

    # == end student section ==

    return phi_new


if __name__ == "__main__":
    print("Test your function here")