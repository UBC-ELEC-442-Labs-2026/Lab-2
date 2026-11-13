import importlib.util
import sys
import time
import os

import numpy as np

import constants
from pal.products.qarm import QArm

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from helpers.RRT_helpers import Cylinder_Obstacle
from helpers.potential_fields_helpers import Potential_Fields_helper


# Import the QArm interface class
interface_directory = os.path.dirname(constants.path_to_interface)

if interface_directory not in sys.path:
    sys.path.append(interface_directory)

from QArm_functions import QArm_Lab_interface #type: ignore

# End of imports


# == Obstacle setup ==

# Choose the obstacle setup: 1 or 2 (see the Obstacle Setup figures in the Lab 2 PDF)
SETUP = 1

if SETUP == 1:
    # One large pillar between the start and the end
    obstacles = np.array([
        Cylinder_Obstacle(np.array([0.65, 0.0, 0.0]), np.array([0.65, 0.0, 1.0]), 0.25),
    ])
    start_pos = np.array([0.5, -0.375, 0.3])
    end_pos = np.array([0.5, 0.375, 0.3])
else:
    # Two short pillars side by side, forming a low wall between the start and the end
    obstacles = np.array([
        Cylinder_Obstacle(np.array([0.45, -0.05, 0.0]), np.array([0.45, -0.05, 0.12]), 0.075),
        Cylinder_Obstacle(np.array([0.45, 0.05, 0.0]), np.array([0.45, 0.05, 0.12]), 0.075),
    ])
    start_pos = np.array([0.275, 0, 0.1])
    end_pos = np.array([0.7, 0, 0.1])

# == end obstacle setup ==


# The start and end positions are converted to joint angles below, with the lab interface's inverse_kinematics()

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
    max_steps = 600 # stop after this many steps (60 s at 10 Hz) if the goal isn't reached
    # --------------------------------------------------------------------------
    # TODO 1: Repulsive force
    # --------------------------------------------------------------------------
    def repulsive_f(rho: float, u_hat: np.ndarray, eta: float, rho_0: float) -> np.ndarray:
        """
        Repulsive force on the end effector from one obstacle.
        See Artificial Potential Fields in the Concept Review of the Lab 2 PDF.

        Parameters
        ----------
        rho : float
            Distance from the end effector to the obstacle's surface, in metres.
        u_hat : np.ndarray, shape (3,)
            Unit vector pointing from the obstacle toward the end effector.
        eta : float
            Repulsive gain.
        rho_0 : float
            Influence distance: an obstacle farther away than this has no effect.

        Returns
        -------
        np.ndarray, shape (3,)
            The repulsive force.
        """
        # == start student section ==
        return np.array([0.0, 0.0, 0.0])
        # ==  end student section  ==

    # --------------------------------------------------------------------------
    # TODO 2: Attractive force
    # --------------------------------------------------------------------------
    def attractive_f(d: float, u_goal: np.ndarray, zeta: float, d_0: float) -> np.ndarray:
        """
        Attractive force on the end effector toward the destination.
        See Artificial Potential Fields in the Concept Review of the Lab 2 PDF.

        Parameters
        ----------
        d : float
            Distance from the end effector to the destination, in metres.
        u_goal : np.ndarray, shape (3,)
            Unit vector pointing from the end effector toward the destination.
        zeta : float
            Attractive gain.
        d_0 : float
            Distance beyond which the attractive force stops growing.

        Returns
        -------
        np.ndarray, shape (3,)
            The attractive force.
        """
        # == start student section ==
        return np.array([0.0, 0.0, 0.0])
        # ==  end student section  ==

    # Distance and direction from the end effector to the destination, stored as a dict
    # with the same keys as each obstacle in obstacle_vecs. You update it in TODO 4.
    destination_vec = {
        "distance": np.linalg.norm(end_pos - start_pos),
        "direction": (end_pos - start_pos) / np.linalg.norm(end_pos - start_pos)
    }

    # --------------------------------------------------------------------------
    # TODO 3: Total force
    # --------------------------------------------------------------------------
    def find_forces(obstacle_vecs: list[dict], destination_vec: dict, max_length: float) -> np.ndarray:
        """
        Total force on the end effector: the repulsive force from every obstacle plus the
        attractive force toward the destination, limited to a length of max_length.

        Parameters
        ----------
        obstacle_vecs : list of dict, one per obstacle (from helpers.get_all_obstacle_distances):
            "distance" : float, distance from the end effector to the obstacle's surface (m)
            "direction" : np.ndarray, shape (3,), unit vector from the obstacle toward the end effector
        destination_vec : dict
            "distance" : float, distance from the end effector to the destination (m)
            "direction" : np.ndarray, shape (3,), unit vector from the end effector toward the destination
        max_length : float
            Largest allowed length of the total force (alpha).

        Returns
        -------
        np.ndarray, shape (3,)
            The total force.
        """
        f = np.zeros(3)
        # == start student section ==
        for obstacle in obstacle_vecs:
            dist = obstacle["distance"]
            direction = obstacle["direction"]
            # TODO: add this obstacle's repulsive force to f

        # TODO: add the attractive force toward the destination (destination_vec has the same keys)

        # TODO: limit the length of f to max_length

        # ==  end student section  ==
        return f

    # --------------------------------------------------------------------------
    # TODO 4: Main loop
    # --------------------------------------------------------------------------
    q = start_phi.copy() # the joint angles sent to the arm

    for step in range(max_steps):
        p4, _ = QArm_Interface.forward_kinematics(q)
        obstacle_vecs = helpers.get_all_obstacle_distances(p4)

        # == start student section ==
        # TODO: update destination_vec["distance"] and destination_vec["direction"] for the current p4

        # ==  end student section  ==

        if destination_vec["distance"] < success_threshold:
            break

        # == start student section ==
        # TODO: find the total force f, turn it into a joint step using the inverse Jacobian, and update q

        # ==  end student section  ==

        QArm_Interface.write_to_arm(q)
        time.sleep(1 / approx_update_freq)

    if destination_vec["distance"] < success_threshold:
        print(f"Reached the destination in {step} steps")
    else:
        print(f"Stopped after {max_steps} steps without reaching the destination")

    time.sleep(4) # Allow arm time to settle at final target position
