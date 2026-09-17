import importlib.util
import sys
from pal.products.qarm import QArm
import numpy as np
import time
import os

from potential_fields_helpers import Potential_Fields_helper
from RRT_helpers import Cylinder_Obstacle
import constants

# Import the QArm interface class
interface_directory = os.path.dirname(constants.path_to_interface)

if interface_directory not in sys.path:
    sys.path.append(interface_directory)

from QArm_functions import QArm_Lab_interface #type: ignore

# End of imports

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
    QArm_Interface = QArm_Lab_interface()
    QArm_Interface.attach_QArm(myArm)
    _, start_phi = QArm_Interface.inverse_kinematics(start_pos, 0, np.array([0, 0, 0, 0]))
    QArm_Interface.write_to_arm(start_phi)
    time.sleep(4)

    # Variable setup : it should work with this set, though you may want to tune it yourself
    zeta = 1

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
    QArm_Interface = QArm_Lab_interface()
    QArm_Interface.attach_QArm(myArm)
    _, start_phi = QArm_Interface.inverse_kinematics(start_pos, 0, np.array([0, 0, 0, 0]))
    QArm_Interface.write_to_arm(start_phi)
    time.sleep(4)

    # Variable setup : it should work with this set, though you may want to tune it yourself
    zeta = 1
    eta = 0.01
    d_0 = 0.3
    rho_0 = 0.05
    alpha = 0.02
    success_threshold = 0.005
    approx_update_freq = 10 # in hz
    # --------------------------------------------------------------------------
    # TODO 1: Implement Repulsive Force Function
    # --------------------------------------------------------------------------
    def repulsive_f(dist, direction, eta, rho):
        """
        Calculates repulsive force vector from an obstacle.
        Refer to Section 3.2 in the Lab 2 Manual.
        
        Inputs:
        - dist: shortest distance from EE to obstacle skin
        - direction: unit vector pointing away from obstacle toward EE
        - eta: repulsive gain parameter
        - rho: influence distance threshold (rho_0)
        """
        # == start student section ==
        return np.array([0.0, 0.0, 0.0])
        # == end student section ==

    # --------------------------------------------------------------------------
    # TODO 2: Implement Attractive Force Function
    # --------------------------------------------------------------------------
    def attractive_f(dist, direction, zeta, d):
        """
        Calculates attractive force vector toward destination.
        Refer to Section 3.1 in the Lab 2 Manual.
        
        Inputs:
        - dist: distance from EE to destination
        - direction: unit vector pointing from EE toward destination
        - zeta: attractive gain parameter
        - d: transition threshold distance (d_0)
        """
        # == start student section ==
        return np.array([0.0, 0.0, 0.0])
        # == end student section ==

    # Helper structure tracking destination vector
    destination_vec = {
        "distance": np.linalg.norm(end_pos - start_pos),
        "direction": (end_pos - start_pos) / np.linalg.norm(end_pos - start_pos)
    }

    # --------------------------------------------------------------------------
    # TODO 3: Implement Total Force Accumulator
    # --------------------------------------------------------------------------
    def find_forces(obstacle_vecs, destination_vec, max_length=1.0):
        """
        Sums repulsive forces across all obstacles and attractive force toward goal.
        Clamps the total force magnitude to max_length (alpha).
        """
        f = np.zeros(3)
        # == start student section ==
        # 1. Loop through obstacle_vecs and accumulate repulsive_f()
        
        # 2. Add attractive_f() towards destination
        
        # 3. Enforce maximum step magnitude cap (alpha)

        # == end student section ==
        return f

    # --------------------------------------------------------------------------
    # TODO 4: Main Potential Field Integration Loop
    # --------------------------------------------------------------------------
    q_new = QArm_Interface.read_from_arm()

    # == start student section ==
    # Implement time-stepping potential field update loop:
    # 1. Compute current EE position p4 via QArm_Interface.forward_kinematics(q_new)
    # 2. Get obstacle vectors via helpers.get_all_obstacle_distances(p4)
    # 3. Update destination_vec distance and direction relative to p4
    # 4. Calculate total 3D force vector using find_forces()
    # 5. Obtain inverse Jacobian: J_inv = QArm_Interface.Inv_Jacobian(q_new)
    # 6. Map 3D workspace force 'f' to 4D joint displacement delta_q
    # 7. Update q_new, send to arm with write_to_arm(q_new), and sleep (1 / approx_update_freq)

    # == end student section ==

    time.sleep(4)  # Allow arm time to settle at final target position
