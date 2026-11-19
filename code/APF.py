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
from helpers.APF_helpers import Potential_Fields_helper


# Import the QArm interface class
interface_directory = os.path.dirname(constants.path_to_interface)

if interface_directory not in sys.path:
    sys.path.append(interface_directory)

from QArm_functions import QArm_Lab_interface #type: ignore

# End of imports


# == Obstacle Setup ==

# Choose the obstacle setup: 1 or 2 (see the Obstacle Setup figures in the Lab 2 PDF)
SETUP = 1

# Heights (z) are measured with respect to the bottom of the arm's base, which sits on a plate 11mm
# above the cardboard playspace, or "table" for short. Thus the table is at z = TABLE_Z0.
TABLE_Z0 = -0.011
# Consequently,
# z = height above the table + TABLE_Z0 and therefore height above the table = z + 0.011

if SETUP == 1:
    # One large pillar (310 mm tall) between the start and the end
    obstacles = np.array([
        Cylinder_Obstacle(np.array([0.65, 0.0, TABLE_Z0]), np.array([0.65, 0.0, TABLE_Z0 + 0.310]), 0.25),
    ])
    start_pos = np.array([0.5, -0.375, 0.05])
    end_pos = np.array([0.5, 0.375, 0.05])
else:
    # Two short pillars (75 mm tall, 104 mm wide octagons) overlapping, forming a barrier
    # between the start and the end
    obstacles = np.array([
        Cylinder_Obstacle(np.array([0.45, -0.05, TABLE_Z0]), np.array([0.45, -0.05, TABLE_Z0 + 0.075]), 0.056),
        Cylinder_Obstacle(np.array([0.45, 0.05, TABLE_Z0]), np.array([0.45, 0.05, TABLE_Z0 + 0.075]), 0.056),
    ])
    start_pos = np.array([0.375, -0.2, 0.03])
    end_pos = np.array([0.6, 0.15, 0.03])

# Wrist angle for the exercise. Puts the gripper's motor and cable on top,
# to keep clear of obstacles.
WRIST_ANGLE = np.radians(159)

# == End Obstacle Setup ==


# The start and end positions are converted to joint angles below, with the lab interface's inverse_kinematics()

joint_limits = np.radians(np.array([
            np.array([-170.0, -85.0, -95.0, -160.0]),
            np.array([ 170.0,  85.0,  75.0,  160.0])
        ]))

# helpers is an instance of Potential_Fields_helper. Call its functions as helpers.function_name(...)
helpers = Potential_Fields_helper(obstacles)

mode = "-1"
while(int(mode) != 0 and int(mode) != 1):
    mode = input("Enter 1 for real hardware, 0 for simulation: ")

with QArm(hardware=int(mode), readMode=0) as myArm:

    # calibration: corrects the commands before sending to hardware
    # hold_on_attach: the arm stays where it is until travel_to() moves it.
    QArm_Interface = QArm_Lab_interface(calibration=True, hold_on_attach=True)
    QArm_Interface.attach_QArm(myArm)

    _, start_phi = QArm_Interface.inverse_kinematics(start_pos, WRIST_ANGLE, np.array([0, 0, 0, 0]))
    _, end_phi = QArm_Interface.inverse_kinematics(end_pos, WRIST_ANGLE, np.array([0, 0, 0, 0]))
    helpers.check_pose(start_phi, start_pos, "start_pos")   # stops with a message if unreachable or colliding
    helpers.check_pose(end_phi, end_pos, "end_pos")

    QArm_Interface.close_gripper()
    QArm_Interface.travel_to(start_phi)   # up, over and down onto the start, clear of the obstacles
    time.sleep(2)   # let the arm settle at the start

    # Variable setup : it should work with this set, though you may want to tune it yourself
    zeta = 1
    eta = 0.01
    d_0 = 0.3
    rho_0 = 0.05
    alpha = 0.01
    success_threshold = 0.005
    approx_update_freq = 8 # in hz
    max_steps = 600 # stop after this many steps (75 s at 8 Hz) if the goal isn't reached
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
        # hint: use np.linalg.norm(v)

        # ==  end student section  ==
        return f

    # --------------------------------------------------------------------------
    # TODO 4: Main loop
    # --------------------------------------------------------------------------
    q = start_phi.copy() # the joint angles sent to the arm
    path = [q.copy()]    # every q sent to the arm, for the plot at the end

    for step in range(max_steps):
        p4, _ = QArm_Interface.forward_kinematics(q)
        obstacle_vecs = helpers.get_all_obstacle_distances(p4)

        # == start student section ==
        # TODO: update destination_vec["distance"] and destination_vec["direction"] for the current p4
        

        # ==  end student section  ==

        if destination_vec["distance"] < success_threshold:
            break

        # == start student section ==
        # TODO: find the total force f with find_forces(), turn it into a joint step using
        # QArm_Interface.Inv_Jacobian(q), and add the step to q.
        # hint: use np.append()

        # ==  end student section  ==

        QArm_Interface.write_to_arm(q)
        path.append(q.copy())
        time.sleep(1 / approx_update_freq)

    if destination_vec["distance"] < success_threshold:
        print(f"Reached the destination in {step} steps")
    else:
        print(f"Stopped after {max_steps} steps without reaching the destination")

    time.sleep(10)   # keep the arm active. Becomes idle and sags after

    # 3D plot of the path the arm took. Close the window to finish
    helpers.plot_path(path)
