import numpy as np
import random
import importlib.util
import sys
import time

from pal.products.qarm import QArm
from RRT_helpers import RRT_Helpers, Cylinder_Obstacle, RRT_Tree_Manager
import constants

# Import the QArm interface class
file_path = constants.path_to_interface
class_name = "QArm_Lab_interface"
module_name = "QArm_Lab_interface_module"
spec = importlib.util.spec_from_file_location(module_name, file_path)
module = importlib.util.module_from_spec(spec)
sys.modules[module_name] = module
spec.loader.exec_module(module)
QArm_Lab_interface = getattr(module, class_name)

# end of imports

# == Obstacle Setup 1 ==
# obstacles = np.array([
#             Cylinder_Obstacle(np.array([0.65, 0.0, 0.0]), np.array([0.65, 0.0, 1.0]), 0.25),
#             ])
# start_pos = np.array([0.5, -0.375, 0.3])
# end_pos = np.array([0.5, 0.375, 0.3])

# == Obstacle Setup 2 ==
# obstacles = np.array([
#             Cylinder_Obstacle(np.array([0.45, -0.25, 0.0]), np.array([0.45, -0.25, 1.0]), 0.1), 
#             Cylinder_Obstacle(np.array([0.3, 0.15, 0.0]), np.array([0.3, 0.15, 0.25]), 0.075), 
#             ])
# start_pos = np.array([0.36, -0.435, 0.09])
# end_pos = np.array([0.54, 0.28, 0.3])

# == Obstacle Setup 3 ==
obstacles = np.array([
            Cylinder_Obstacle(np.array([0.45, -0.05, 0.0]), np.array([0.45, -0.05, 0.12]), 0.075), 
            Cylinder_Obstacle(np.array([0.45, 0.05, 0.0]), np.array([0.45, 0.05, 0.12]), 0.075), 
            ])
start_pos = np.array([0.275, 0, 0.1])
end_pos = np.array([0.7, 0, 0.1])


# == end obstacle setup ==

#find start and end positions in joint space, using your geometric inverse kinematic function
# start_phi = geometric_inv_kin(start_pos, 0, np.radians(np.array([-45.0, 50.0, -43.0, 0.0])))
# destination_phi = geometric_inv_kin(end_pos, 0, np.radians(np.array([45.0, 50.0, -43.0, 0.0])))

joint_limits = np.radians(np.array([
            np.array([-170.0, -85.0, -95.0, -160.0]),
            np.array([ 170.0,  85.0,  75.0,  160.0])
        ]))

helpers = RRT_Helpers(obstacles)


mode = "-1"
while(int(mode) != 0 and int(mode) != 1):
    mode = input("Enter 1 for real hardware, 0 for simulation: ")


with QArm(hardware=int(mode), readMode=0) as myArm:

    print("here")

    QArm_interface = QArm_Lab_interface()
    QArm_interface.attach_QArm(myArm)

    _, start_phi = QArm_interface.inverse_kinematics(start_pos, 0, np.array([0, 0, 0, 0]))
    _, destination_phi = QArm_interface.inverse_kinematics(end_pos, 0, np.array([0, 0, 0, 0]))

    tree = RRT_Tree_Manager(start_phi)

    QArm_interface.write_to_arm(start_phi)
    time.sleep(4)

    #TODO: The rest is up to the students:

