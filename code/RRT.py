import importlib.util
import random
import sys
import time
import os

import numpy as np

import constants
from pal.products.qarm import QArm

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from helpers.RRT_helpers import Cylinder_Obstacle, RRT_Helpers, RRT_Tree_Manager


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

# Joint limits [base, shoulder, elbow, wrist] in radians. The wrist's range is set to just WRIST_ANGLE,
# so random configurations keep the wrist there.
joint_limits = np.radians(np.array([
            np.array([-170.0, -85.0, -95.0, 0.0]),
            np.array([ 170.0,  85.0,  75.0, 0.0])
        ]))
joint_limits[:, 3] = WRIST_ANGLE

# helpers is an instance of RRT_Helpers. Call its functions as helpers.function_name(...)
helpers = RRT_Helpers(obstacles)

mode = "-1"
while(int(mode) != 0 and int(mode) != 1):
    mode = input("Enter 1 for real hardware, 0 for simulation: ")


with QArm(hardware=int(mode), readMode=0) as myArm:

    # calibration: corrects the commands before sending to hardware
    # hold_on_attach: the arm stays where it is until travel_to() moves it.
    QArm_interface = QArm_Lab_interface(calibration=True, hold_on_attach=True)
    QArm_interface.attach_QArm(myArm)

    _, start_phi = QArm_interface.inverse_kinematics(start_pos, WRIST_ANGLE, np.array([0, 0, 0, 0]))
    _, destination_phi = QArm_interface.inverse_kinematics(end_pos, WRIST_ANGLE, np.array([0, 0, 0, 0]))
    helpers.check_pose(start_phi, start_pos, "start_pos")   # stops with a message if unreachable or colliding
    helpers.check_pose(destination_phi, end_pos, "end_pos")

    # tree is an instance of RRT_Tree_Manager, starting at start_phi. Attributes are tree.nodes and tree.parents.
    # Methods include tree.add_node(...), tree.get_nearest_neighbor(...) and tree.get_size()
    tree = RRT_Tree_Manager(start_phi)

    QArm_interface.close_gripper()
    QArm_interface.travel_to(start_phi)   # up, over and down onto the start, clear of the obstacles
    time.sleep(2)   # let the arm settle at the start

    # --------------------------------------------------------------------------
    # TODO 1: Implement Goal-Biased Random Joint Sampling
    # --------------------------------------------------------------------------
    def rand_point(goal_phi: np.ndarray, bias: float = 0.1) -> np.ndarray:
        """
        Pick a random configuration for the tree to grow toward.

        Parameters
        ----------
        goal_phi : np.ndarray, shape (4,)
            The goal configuration.
        bias : float
            Chance of returning goal_phi instead of a random configuration.

        Returns
        -------
        np.ndarray, shape (4,)
        """
        # == start student section ==
        # With probability bias, return goal_phi. Otherwise, pick a random angle for each joint
        # between its limits (joint_limits[0] has the minimums, joint_limits[1] the maximums).
        # np.random.random() and np.random.uniform(low, high) will help here.
        probability = np.random.random()
        die_outcome = round(np.random.uniform(0.5, 6.5))
        pass
        # == end student section ==

    # --------------------------------------------------------------------------
    # TODO 2: Implement Small Step Function
    # --------------------------------------------------------------------------
    def small_step(q_near: np.ndarray, q_rand: np.ndarray, step_size: float = 0.1) -> np.ndarray:
        """
        Take one step from q_near toward q_rand, in joint space.

        Parameters
        ----------
        q_near : np.ndarray, shape (4,)
            Where to step from.
        q_rand : np.ndarray, shape (4,)
            The configuration to step toward.
        step_size : float
            Length of the step, in radians.

        Returns
        -------
        np.ndarray, shape (4,)
            The new configuration.
        """
        # == start student section ==
        # Move step_size radians from q_near toward q_rand
        # hint: use np.linalg.norm(v)
        pass
        # == end student section ==

    # --------------------------------------------------------------------------
    # TODO 3: Implement Backtracking Function
    # --------------------------------------------------------------------------
    def backtrack(node_idx: int) -> list[np.ndarray]:
        """
        Follow the tree from node node_idx back to the root (the start).

        Returns
        -------
        list of np.ndarray
            Joint configurations, starting with node node_idx and ending with the root.
        """
        # == start student section ==
        path = []
        while node_idx is not None:
            # TODO: add this node's configuration to path, then move to its parent
            pass
        return path
        # == end student section ==

    # --------------------------------------------------------------------------
    # TODO 4: Main RRT Loop
    # --------------------------------------------------------------------------
    # Feel free to tune these for smoother motion
    step_size = 0.05  # in radians
    success_threshold = step_size * 3
    max_iterations = 10000

    path_found = False

    for i in range(max_iterations):
        # == start student section ==
        # TODO: one RRT iteration. Sample a configuration, find the nearest node in the tree,
        # take a small step toward the sample, and add the new configuration to the tree if it
        # is collision-free. If you added it and it is within success_threshold of destination_phi,
        # set path_found = True and stop the loop with break.
        pass
        # == end student section ==

    # --------------------------------------------------------------------------
    # Path Execution on Arm (Provided)
    # --------------------------------------------------------------------------
    if not path_found:
        q_nearest, _ = tree.get_nearest_neighbor(destination_phi)
        print(f"No path found after {max_iterations} iterations. Tree size: {tree.get_size()}")
        print(f"Closest configuration to the goal: {q_nearest}")
        sys.exit()

    path = backtrack(tree.get_size() - 1)   # from the last node added (near the goal) back to the start
    path.reverse()                          # flip the nodes to go from the start to the goal

    # Finish exactly on the goal (check_pose already made sure the goal is collision-free)
    path.append(destination_phi)

    print(f"Path found! {len(path)} tree nodes")

    # Straighten the path. Uncomment to remove the wiggle
    # path = helpers.shortcut_path(path, step_size)
    
    # 3D plot of the plan. Close the window to run it on the arm
    helpers.plot_path(path)

    for node in path:
        QArm_interface.write_to_arm(node)
        time.sleep(0.3)

    time.sleep(10)   # keep the arm active. Becomes idle and sags after

