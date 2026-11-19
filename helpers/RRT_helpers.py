import sys

import numpy as np
from scipy.spatial import KDTree

from pal.products.qarm import QArm
from hal.products.qarm import QArmUtilities


class Cylinder_Obstacle:

    '''
    Represents a single cylindrical obstacle
    '''

    def __init__(self, Q1, Q2, R):
        '''
        Q1 to Q2 is the center line of the cylinder with radius R
        '''
        self.Q1 = Q1
        self.Q2 = Q2
        self.R = R

    def get_obs(self):
        """The obstacle as [Q1, Q2, R]."""
        return np.array([self.Q1, self.Q2, self.R])



class RRT_Helpers:

    '''
    Helper functions for RRT impelementation
    '''

    L_1 = 0.1400
    L_2 = 0.3500
    L_3 = 0.0500
    L_4 = 0.2500
    L_5 = 0.1500
    BETA = np.arctan(L_3/L_2)

    LAMBDA_1 = L_1
    LAMBDA_2 = np.sqrt(L_2**2 + L_3**2)
    LAMBDA_3 = L_4 + L_5

    # Collision model (measured on the lab arms, Nov 2026). Radii in metres.
    LINK_RADII = {
        "base": 0.07,          # base column, table to shoulder
        "upper_arm": 0.04,     # shoulder to elbow
        "forearm": 0.03,       # elbow to the back of the gripper
        "gripper_body": 0.05,  # the gripper motor housing
        "fingers": 0.012,      # the closed fingers
    }
    GRIPPER_BODY_END = 0.148   # m back from the fingertip where the gripper body starts
    FINGERS_END = 0.086        # m back from the fingertip where the fingers start
    SAFETY_MARGIN = 0.01       # extra gap every part of the arm keeps from the obstacles (m)
    MIN_TIP_Z0 = 0.011         # lowest fingertip height (m, frame {0}); same as QArm_calibration.MIN_TIP_Z0

    def __init__(self, obstacles):
        """obstacles: list of Cylinder_Obstacle."""
        self.myArmUtilities = QArmUtilities()
        self.joint_limits = np.radians(np.array([
            np.array([-170.0, -85.0, -95.0, -160.0]),
            np.array([ 170.0,  85.0,  75.0,  160.0])
        ]))

        # self.obstables = np.array([
        #     Cylinder_Obstacle(np.array([0.6, 0.0, 0.0]), np.array([0.6, 0.0, 0.3]), 0.15), # hypotehtical obstacle
        #     ])
        self.obstacles = obstacles
        
    def get_joint_positions(self, phi):
        """
        Positions of the arm's joints for joint angles phi, in frame {0}.

        Returns
        -------
        list of 5 np.ndarray, shape (3,)
            [base, shoulder, elbow, wrist frame {3}, end effector frame {4}]
        """

        def quanser_arm_DH(a, alpha, d, theta):
            """ QUANSER_ARM_DH
            v 1.0 - 26th March 2019

            REFERENCE:
            Chapter 3. Forward and Inverse Kinematics
            Robot Modeling and Control
            Spong, Hutchinson, Vidyasagar
            2006

            INPUTS:
            a       :   translation  : along : x_{i}   : from : z_{i-1} : to : z_{i}
            alpha   :      rotation  : about : x_{i}   : from : z_{i-1} : to : z_{i}
            d       :   translation  : along : z_{i-1} : from : x_{i-1} : to : x_{i}
            theta   :      rotation  : about : z_{i-1} : from : x_{i-1} : to : x_{i}
            (Standard DH Parameters are being used here)

            OUTPUTS:
            T       : transformation                   : from :     {i} : to : {i-1}"""

            # Rotation Transformation about z axis by theta
            T_R_z = np.array([[np.cos(theta), -np.sin(theta), 0, 0], [np.sin(theta), np.cos(theta), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float64)

            # Translation Transformation along z axis by d
            T_T_z = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d], [0, 0, 0, 1]], dtype=np.float64)

            # Translation Transformation along x axis by a
            T_T_x = np.array([[1, 0, 0, a], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float64)

            # Rotation Transformation about x axis by alpha
            T_R_x = np.array([[1, 0, 0, 0], [0, np.cos(alpha), -np.sin(alpha), 0], [0, np.sin(alpha), np.cos(alpha), 0], [0, 0, 0, 1]], dtype=np.float64)

            # For a transformation FROM frame {i} TO frame {i-1}: A
            T = T_R_z@T_T_z@T_T_x@T_R_x

            return T

        # From phi space to theta space
        theta = phi.copy()
        theta[0] = phi[0]
        theta[1] = phi[1] + self.BETA - np.pi/2
        theta[2] = phi[2] - self.BETA
        theta[3] = phi[3]

        # Transformation matrices for all frames:

        # T{i-1}{i} = quanser_arm_DH(  a, alpha,  d,     theta )
        T01 = quanser_arm_DH(            0, -np.pi/2, self.LAMBDA_1,  theta[0] )
        T12 = quanser_arm_DH( self.LAMBDA_2,        0,            0,  theta[1] )
        T23 = quanser_arm_DH(            0, -np.pi/2,            0,  theta[2] )
        T34 = quanser_arm_DH(            0,        0, self.LAMBDA_3,  theta[3] )

        T02 = T01@T12
        T03 = T02@T23
        T04 = T03@T34

        # Position of end-effector Transformation

        # Extract the Position vector
        p0   = np.array([0, 0, 0])
        p1   = T01[0:3,3];
        p2   = T02[0:3,3];
        p3   = T03[0:3,3];
        p4   = T04[0:3,3];

        joint_positions = [p0, p1, p2, p3, p4]

        return joint_positions
    
    def _segment_to_cylinder(self, P1, P2, obst, spacing=0.005):
        """
        Distance from the line segment P1-P2 to the surface of a solid, flat-topped cylinder
        (0 if they overlap). Checks points every `spacing` metres along the segment, so it can
        read up to spacing/2 too far.
        """
        # Points along the segment, one row per point
        n = int(np.ceil(np.linalg.norm(P2 - P1) / spacing)) + 1
        points = P1 + np.linspace(0, 1, n)[:, None] * (P2 - P1)

        # The cylinder's axis: unit direction k and length
        axis = np.asarray(obst.Q2, dtype=float) - np.asarray(obst.Q1, dtype=float)
        length = np.linalg.norm(axis)
        k = axis / length

        # Each point's height along the axis, and its distance from the axis
        rel = points - obst.Q1
        h = rel @ k                                          # @ is the dot product
        rho = np.linalg.norm(rel - h[:, None] * k, axis=1)

        # How far each point is outside the cylinder (0 if inside that direction)
        outside_ends = np.maximum(np.maximum(-h, h - length), 0)   # below the bottom or above the top
        outside_wall = np.maximum(rho - obst.R, 0)                 # outside the side wall

        # Closest point wins
        return float(np.min(np.hypot(outside_ends, outside_wall)))

    def get_links(self, phi):
        """
        The arm as line segments, each with the radius of the tube around it.

        Returns
        -------
        list of (name, start point, end point, radius)
        """
        p = self.get_joint_positions(np.asarray(phi, dtype=float))
        tip = p[4]                                # frame {4}, taken as the fingertip
        u = (p[4] - p[2]) / self.LAMBDA_3         # unit vector along the forearm and gripper

        def back(d):
            """The point d metres back from the tip, along the gripper."""
            return tip - d * u

        body_start = back(self.GRIPPER_BODY_END)
        fingers_start = back(self.FINGERS_END)
        r_fingers = self.LINK_RADII["fingers"]
        fingers_end = back(r_fingers)   # pulled back by its radius so the rounded end stops at the tip

        return [
            ("base",         p[0],          p[1],          self.LINK_RADII["base"]),
            ("upper_arm",    p[1],          p[2],          self.LINK_RADII["upper_arm"]),
            ("forearm",      p[2],          body_start,    self.LINK_RADII["forearm"]),
            ("gripper_body", body_start,    fingers_start, self.LINK_RADII["gripper_body"]),
            ("fingers",      fingers_start, fingers_end,   r_fingers),
        ]

    def is_configuration_colliding(self, phi: np.ndarray) -> bool:
        """
        Check a joint configuration against the joint limits, the floor and every obstacle.

        Parameters
        ----------
        phi : np.ndarray, shape (4,)
            Joint angles [base, shoulder, elbow, wrist], in radians.

        Returns
        -------
        bool
            True if the arm breaks a joint limit, goes below the floor or comes within
            SAFETY_MARGIN of an obstacle, False if it's safe.
        """
        if self.is_outside_joint_limits(phi):
            return True  # joint angles are out of bounds

        links = self.get_links(phi)

        # The fingertip: the end of the fingers segment, plus the radius it was pulled back by
        _, fingers_start, fingers_end, r_fingers = links[-1]
        u = (fingers_end - fingers_start) / np.linalg.norm(fingers_end - fingers_start)
        tip = fingers_end + r_fingers * u
        if tip[2] < self.MIN_TIP_Z0:   # too close to the table
            return True

        # Every link against every obstacle
        for name, P1, P2, r_link in links:
            for obst in self.obstacles:
                if self._segment_to_cylinder(P1, P2, obst) < r_link + self.SAFETY_MARGIN:
                    return True  # collision detected

        return False  # safe configuration

    def check_pose(self, phi: np.ndarray, target_pos: np.ndarray, name: str) -> None:
        """
        Stop the program with a message if phi doesn't put the tip at target_pos (the inverse
        kinematics failed), or if the arm would collide there. Use it on the start and the goal.
        """
        tip = self.get_joint_positions(np.asarray(phi, dtype=float))[4]

        # Quanser's inverse_kinematics returns [0, 0, 0, 0] when it finds no valid solution
        if np.linalg.norm(tip - target_pos) > 0.01:
            sys.exit(f"{name} = {target_pos} can't be reached by the arm. Move it.")

        if self.is_configuration_colliding(phi):
            sys.exit(f"At {name} = {target_pos} the arm would hit an obstacle, the table or a "
                     f"joint limit (in the collision model). Move it.")

    def shortcut_path(self, path: list[np.ndarray], step_size: float = 0.1) -> list[np.ndarray]:
        """
        Remove the random detours from an RRT path: from each point, jump straight to the
        furthest later point that can be reached without a collision. The result is split back
        into steps of at most step_size radians, so the arm never takes a big jump.
        """
        def segment_is_free(qa, qb):
            """True if the straight joint-space move qa -> qb is collision-free (checked every 0.02 rad)."""
            n = max(1, int(np.ceil(np.linalg.norm(qb - qa) / 0.02)))
            for k in range(1, n + 1):
                if self.is_configuration_colliding(qa + (qb - qa) * k / n):
                    return False
            return True

        path = [np.asarray(q, dtype=float) for q in path]

        # Shortcut: from point i, find the furthest point j we can go to directly
        short = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not segment_is_free(path[i], path[j]):
                j -= 1
            short.append(path[j])
            i = j

        # Split the long straight moves back into small steps
        steps = [short[0]]
        for qa, qb in zip(short[:-1], short[1:]):   # each pair of neighbouring points
            n = max(1, int(np.ceil(np.linalg.norm(qb - qa) / step_size)))
            for k in range(1, n + 1):
                steps.append(qa + (qb - qa) * k / n)
        return steps

    def plot_path(self, path: list[np.ndarray], title: str = "") -> None:
        """
        3D plot of the obstacles, the arm at the start (green) and the end (red) of the path,
        and the line the fingertip follows (orange). Close the window to carry on.
        """
        import matplotlib.pyplot as plt

        fig = plt.figure(figsize=(8, 7))
        ax = fig.add_subplot(projection="3d")

        # Obstacles
        theta = np.linspace(0, 2 * np.pi, 40)
        for obst in self.obstacles:
            Q1 = np.asarray(obst.Q1, dtype=float)
            Q2 = np.asarray(obst.Q2, dtype=float)

            # Two unit vectors a, b perpendicular to the cylinder's axis k
            k = (Q2 - Q1) / np.linalg.norm(Q2 - Q1)
            if abs(k[0]) < 0.9:
                a = np.cross(k, [1, 0, 0])
            else:
                a = np.cross(k, [0, 1, 0])
            a /= np.linalg.norm(a)
            b = np.cross(k, a)

            def ring(c, r):
                """Points on a circle of radius r around centre c, perpendicular to the axis."""
                return c + r * (np.outer(np.cos(theta), a) + np.outer(np.sin(theta), b))

            # Outline of the bottom and top
            for c in (Q1, Q2):
                circle = ring(c, obst.R)
                ax.plot(circle[:, 0], circle[:, 1], circle[:, 2], color="gray", lw=1)

            # Side wall and top as see-through surfaces
            side = np.stack([ring(Q1, obst.R), ring(Q2, obst.R)])
            ax.plot_surface(side[..., 0], side[..., 1], side[..., 2], color="gray", alpha=0.3, linewidth=0)
            top = np.stack([ring(Q2, 0), ring(Q2, obst.R)])
            ax.plot_surface(top[..., 0], top[..., 1], top[..., 2], color="gray", alpha=0.3, linewidth=0)

        def draw_arm(phi, color, alpha=1.0, lw=4, label=None):
            for i, (name, P1, P2, r) in enumerate(self.get_links(phi)):
                ax.plot([P1[0], P2[0]], [P1[1], P2[1]], [P1[2], P2[2]],
                        color=color, alpha=alpha, lw=lw, solid_capstyle="round",
                        label=label if i == 0 else None)   # label only once per arm

        # About 8 faint arms along the way, then the start and the end
        every = max(1, len(path) // 8)
        for q in path[1:-1:every]:   # every `every`-th point, skipping the first and last
            draw_arm(q, "tab:blue", alpha=0.15, lw=2)
        draw_arm(path[0], "tab:green", label="arm at the start")
        draw_arm(path[-1], "tab:red", label="arm at the end")

        # The line the fingertip follows
        tips = np.array([self.get_joint_positions(np.asarray(q, dtype=float))[4] for q in path])
        ax.plot(tips[:, 0], tips[:, 1], tips[:, 2], color="tab:orange", lw=2, label="fingertip path")

        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_zlabel("z (m)")
        ax.set_xlim(-0.1, 0.9)
        ax.set_ylim(-0.5, 0.5)
        ax.set_zlim(-0.011, 0.8)
        ax.set_box_aspect((1.0, 1.0, 0.811))   # same scale on all three axes
        ax.set_title(title or "Planned path (close this window to run it)", fontsize=10)
        ax.legend(loc="upper left", fontsize=8)

        print("A 3D plot has opened. It may be behind other windows or only in the taskbar. "
              "Close it to continue.")
        plt.show()

    def is_outside_joint_limits(self, phi):
        '''
        Checks the joint angles against the joint limits

        Returns: True if any joint is outside its limits, False otherwise
        '''
        for i in range(4):
            if phi[i] < self.joint_limits[0][i] or phi[i] > self.joint_limits[1][i]:
                return True
        
        return False
    

class RRT_Tree_Manager:
    """
    Stores the RRT tree and finds nearest nodes.

    The tree is two lists of the same length: nodes[i] is node i's joint configuration,
    and parents[i] is the index of node i's parent. The root (the start) is node 0, and
    its parent is None.
    """
    def __init__(self, start_phi):
        """Start a tree with one node, the root: start_phi."""
        # nodes[i] is node i's joint configuration
        self.nodes: list[np.ndarray] = [np.array(start_phi)]
        # parents[i] is the index of node i's parent (None for the root)
        self.parents: list[int | None] = [None]

    def add_node(self, q_new: np.ndarray, parent_index: int) -> None:
        """Add q_new to the tree as a child of node parent_index."""
        self.nodes.append(np.array(q_new))
        self.parents.append(parent_index)

    def get_nearest_neighbor(self, q_rand: np.ndarray) -> tuple[np.ndarray, int]:
        """
        The node closest to q_rand in joint space.

        Returns
        -------
        node : np.ndarray, shape (4,)
            The closest node's joint configuration.
        node_idx : int
            Its index in the tree.
        """
        spatial_tree = KDTree(np.array(self.nodes))
        _, nearest_idx = spatial_tree.query(q_rand)
        return self.nodes[nearest_idx], nearest_idx
        
    def get_size(self) -> int:
        """Number of nodes in the tree."""
        return len(self.nodes)
