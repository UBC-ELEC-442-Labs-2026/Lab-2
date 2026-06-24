import importlib.util
import sys
from pal.products.qarm import QArm
import numpy as np
import time

from potential_fields_helpers import Potential_Fields_helper
from RRT_helpers import Cylinder_Obstacle
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

# == Start Obstacle setup (choose one) ==

# == Obstacle Setup 1 ==
obstacles = np.array([
            Cylinder_Obstacle(np.array([0.65, 0.0, 0.0]), np.array([0.65, 0.0, 1.0]), 0.25),
            ])
start_pos = np.array([0.5, -0.375, 0.3])
end_pos = np.array([0.5, 0.375, 0.3])

# == Obstacle Setup 2 ==
# obstacles = np.array([
#             Cylinder_Obstacle(np.array([0.45, -0.25, 0.0]), np.array([0.45, -0.25, 1.0]), 0.1), 
#             Cylinder_Obstacle(np.array([0.3, 0.15, 0.0]), np.array([0.3, 0.15, 0.25]), 0.075), 
#             ])
# start_pos = np.array([0.36, -0.435, 0.09])
# end_pos = np.array([0.54, 0.28, 0.3])

# == Obstacle Setup 3 ==
# obstacles = np.array([
#             Cylinder_Obstacle(np.array([0.45, -0.05, 0.0]), np.array([0.45, -0.05, 0.12]), 0.075), 
#             Cylinder_Obstacle(np.array([0.45, 0.05, 0.0]), np.array([0.45, 0.05, 0.12]), 0.075), 
#             ])
# start_pos = np.array([0.275, 0, 0.1])
# end_pos = np.array([0.7, 0, 0.1])

# == end obstacle setup ==

joint_limits = np.radians(np.array([
            np.array([-170.0, -85.0, -95.0, -160.0]),
            np.array([ 170.0,  85.0,  75.0,  160.0])
        ]))


helpers = Potential_Fields_helper(obstacles)

mode = "-1"
while(int(mode) != 0 and int(mode) != 1):
    mode = input("Enter 1 for real hardware, 0 for simulation: ")

with QArm(hardware=int(mode), readMode=0) as myArm:
    QArm_Interface = QArm_Lab_interface(myArm)
    _, start_phi = QArm_Interface.inverse_kinematics(start_pos, 0, np.array([0, 0, 0, 0]))
    QArm_Interface.write_to_arm(start_phi, 1.0)
    time.sleep(4)

    #TODO: The rest is up to the students:
    # this works but probably could use some tuning
    zeta = 0.5
    eta = 0.05
    d_0 = 0.5
    rho_0 = 0.2
    alpha = 0.05
    success_threshold = 0.1

    def repulsive_f(dist, dir, eta, rho):
        # dir should point away from the obstacle (obstacle to EE)
        if dist <= rho:
            return ( eta * (1/dist - 1/rho) * (1/dist**2) ) * dir
        return np.array([0, 0, 0])
    
    def attractive_f(dist, dir, zeta, d):
        # dir should point toward the destination (EE to destination)
        if dist < d:
            return ( zeta * dist ) * dir
        return ( zeta * d ) * dir

    destination_vec = {
            "distance": np.linalg.norm(end_pos - start_pos),
            "direction": (end_pos - start_pos) / np.linalg.norm(end_pos - start_pos)
        }

    def find_forces(obstacle_vecs, destination_vec):
        f = np.zeros(3)
        for o in obstacle_vecs:
            f += repulsive_f(o["distance"], o["direction"], eta, rho_0)
        f += attractive_f(destination_vec["distance"], destination_vec["direction"], zeta, d_0)
            
        return f

    while destination_vec["distance"] > success_threshold:
        q_now = QArm_Interface.read_from_arm()
        p4, _ = QArm_Interface.forward_kinematics(q_now)

        obstacle_vecs = helpers.get_all_obstacle_distances(p4)
        destination_vec["distance"] = np.linalg.norm(end_pos - p4)
        destination_vec["direction"] = (end_pos - p4) / np.linalg.norm(end_pos - p4)

        f = find_forces(obstacle_vecs, destination_vec)
        print(f)

        J = QArm_Interface.Jacobian(q_now)
        tau = J.T @ np.append(f, 0)

        print(f"obstacle vector: {obstacle_vecs[0]['direction']}")
        print(f"destination vector: {destination_vec['direction']}")
        print(f"force: {f}")

        q_new = q_now + alpha * (tau/np.linalg.norm(tau))

        QArm_Interface.write_to_arm(q_new, 1)
        time.sleep(0.2)


