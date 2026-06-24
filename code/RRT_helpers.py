from pal.products.qarm import QArm
from hal.products.qarm import QArmUtilities
import numpy as np
from scipy.spatial import KDTree

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

    def __init__(self, obstacles):
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
    
    def min_segment_distance(self, P1, P2, Q1, Q2):
        """
        Computes the minimum distance between two 3D line segments:
        Segment 1: from P1 to P2
        Segment 2: from Q1 to Q2
        """
        u = P2 - P1
        v = Q2 - Q1
        w0 = P1 - Q1

        a = np.dot(u, u)
        b = np.dot(u, v)
        c = np.dot(v, v)
        d = np.dot(u, w0)
        e = np.dot(v, w0)

        denominator = a * c - b * b

        # If lines are parallel, handle division by zero
        if denominator < 1e-6:
            s_raw = 0.0
            t_raw = e / c if c != 0 else 0.0
        else:
            s_raw = (b * e - c * d) / denominator
            t_raw = (a * e - b * d) / denominator

        # Clamp parameters to the finite segment bounds [0, 1]
        s = np.clip(s_raw, 0.0, 1.0)
        t = np.clip(t_raw, 0.0, 1.0)

        # Recompute closest points given clamped parameters
        P_closest = P1 + s * u
        Q_closest = Q1 + t * v

        # Return the Euclidean distance between the closest points
        return np.linalg.norm(P_closest - Q_closest)
        
    def is_configuration_colliding(self, phi):

        # check joint limits
        if self.check_self_collision(phi):
            return True # joint angles are out of bounds
        
        # Compute DH Forward Kinematics to get all joint 3D positions
        p = self.get_joint_positions(phi) 
        link_radii = [0.07, 0.07, 0.07]
        links = [
            [p[0], p[1]], # base to shoulder
            [p[1], p[2]], # shoulder to elbow
            [p[2], p[4]] # elbow to EE
        ]
        num_links = len(links)
        
        # Check every link against every obstacle
        for i in range(num_links):
            P1 = links[i][0]     # Start of link i
            P2 = links[i][1]   # End of link i
            r_link = link_radii[i]
            
            # --- Robot vs Environment ---
            for obst in self.obstacles:
                # obst contains (Q1, Q2, r_obst)
                d = self.min_segment_distance(P1, P2, obst.Q1, obst.Q2)
                if d < (r_link + obst.R):
                    return True # Instant fail, collision detected!
                    
        return False # Safe configuration!

    def check_self_collision(self, phi):
        '''
        Checks if the arm is in a valid position

        Returns: True if the arm collides (invalid position), false otherwise
        '''
        for i in range(4):
            if phi[i] < self.joint_limits[0][i] or phi[i] > self.joint_limits[1][i]:
                return True
        
        return False
    

class RRT_Tree_Manager:
    '''
    Abstracted spatial tree manager for student labs.
    Handles node storage and rapid nearest-neighbor lookup via SciPy.
    '''
    def __init__(self, start_phi):
        # A flat list to store node joint matrices: shape (N, 4)
        self.nodes = [np.array(start_phi)]
        # Parallel list to map the parent index for path tracing back to root - ie. node i's parent is self.parents[i]
        self.parents = [None]

    def add_node(self, q_new, parent_index):
        """Adds a verified configuration and links its parent index."""
        self.nodes.append(np.array(q_new))
        self.parents.append(parent_index)

    def get_nearest_neighbor(self, q_rand):
        """Returns (nearest_configuration, nearest_index) quickly via KD-Tree."""
        spatial_tree = KDTree(np.array(self.nodes))
        _, nearest_idx = spatial_tree.query(q_rand)
        return self.nodes[nearest_idx], nearest_idx
        
    def get_size(self):
        return len(self.nodes)