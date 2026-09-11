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

    # --------------------------------------------------------------------------
    # TODO 1: Implement Goal-Biased Random Joint Sampling
    # --------------------------------------------------------------------------
    def rand_point(goal_phi, bias=0.1):
        """
        Generates a random joint-space sample q_rand.
        
        Requirements:
        - With probability `bias`, return `goal_phi` (goal biasing).
        - Otherwise, sample each joint uniformly within `joint_limits`.
        """
        # == start student section ==
        pass
        # == end student section ==

    # --------------------------------------------------------------------------
    # TODO 2: Implement Small Step Function
    # --------------------------------------------------------------------------
    def small_step(q_near, q_rand, step_size=0.1):
        """
        Computes configuration q_new starting from q_near and stepping distance
        `step_size` towards q_rand in joint space.
        """
        # == start student section ==
        pass
        # == end student section ==

    # --------------------------------------------------------------------------
    # TODO 3: Implement Backtracking Function
    # --------------------------------------------------------------------------
    def backtrack(parent_idx):
        """
        Traces tree nodes from parent_idx back to the root (where parent index is None).
        Returns the ordered list of configuration nodes.
        """
        # == start student section ==
        path = []
        return path
        # == end student section ==

    # --------------------------------------------------------------------------
    # TODO 4: Main RRT Loop
    # --------------------------------------------------------------------------
    step_size = 0.1  # in radians
    success_threshold = step_size * 3
    path_found = False
    max_iterations = 10000

    # == start student section ==
    # Implement the RRT algorithm loop:
    # 1. Sample q_rand via rand_point()
    # 2. Find nearest neighbor using tree.get_nearest_neighbor(q_rand)
    # 3. Compute q_new using small_step()
    # 4. Check obstacle collision using helpers.is_configuration_colliding(q_new)
    # 5. If safe, add q_new to tree using tree.add_node(q_new, parent_index=near_idx)
    # 6. Stop when q_new is within success_threshold of destination_phi

    # == end student section ==

    # --------------------------------------------------------------------------
    # Path Execution on Arm (Provided)
    # --------------------------------------------------------------------------
    if not path_found:
        q_nearest, _ = tree.get_nearest_neighbor(destination_phi)
        print("No path found. Nearest configuration: {}".format(q_nearest))
        print(f"Tree size: {tree.get_size()}")
        sys.exit()

    path = backtrack(near_idx)
    print(f"Path found! Steps taken: {len(path)}")

    # Execute path on the arm (in forward order from root to destination)
    for node in reversed(path):
        QArm_interface.write_to_arm(node)
        time.sleep(0.2)

