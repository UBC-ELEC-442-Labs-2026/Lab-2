import numpy as np

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
        return np.array([self.Q1, self.Q2, self.R])
    


    
class Potential_Fields_helper:

    def __init__(self, obstacles):
        self.myArmUtilities = QArmUtilities()
        self.joint_limits = np.radians(np.array([
            np.array([-170.0, -85.0, -95.0, -160.0]),
            np.array([ 170.0,  85.0,  75.0,  160.0])
        ]))
        self.obstacles = obstacles


    def get_ee_to_obstacle_vector(self, p4, obst):
        """
        Distance and direction from the end effector (p4) to a solid, flat-topped cylinder.

        Parameters:
        - p4: np.array([x, y, z]) of the end-effector
        - obst: Cylinder_Obstacle object containing Q1, Q2, and R

        Returns:
        - distance_to_skin: Float, shortest distance from the EE to the cylinder's surface
                            (side wall, top or bottom); 0 if the EE is inside it
        - direction_vector: np.array([x, y, z]), unit vector from the closest point on the
                            surface toward p4, in {0} coordinates
        """
        Q1 = np.asarray(obst.Q1, dtype=float)
        axis = np.asarray(obst.Q2, dtype=float) - Q1
        length = np.linalg.norm(axis)
        k = axis / length

        rel = np.asarray(p4, dtype=float) - Q1
        h = np.dot(rel, k)                       # height along the cylinder's axis
        radial = rel - h * k                     # sideways offset from the axis
        rho = np.linalg.norm(radial)
        radial_dir = radial / rho if rho > 1e-9 else np.cross(k, [0.0, 0.0, 1.0] if abs(k[2]) < 0.9 else [1.0, 0.0, 0.0])
        radial_dir = radial_dir / np.linalg.norm(radial_dir)

        # Closest point on the solid cylinder: clamp the height to [0, length] and the radius to R
        closest = Q1 + np.clip(h, 0.0, length) * k + min(rho, obst.R) * radial_dir
        vector_to_ee = np.asarray(p4, dtype=float) - closest
        distance_to_skin = np.linalg.norm(vector_to_ee)

        if distance_to_skin < 1e-9:
            # Inside the cylinder: push out through the nearest face (the side wall or the top)
            if (obst.R - rho) < (length - h):
                return 0.0, radial_dir
            return 0.0, k

        return distance_to_skin, vector_to_ee / distance_to_skin
    

    
    def get_all_obstacle_distances(self, p4: np.ndarray) -> list[dict]:
        """
        Distance and direction from the end effector to every obstacle.

        Parameters
        ----------
        p4 : np.ndarray, shape (3,)
            End-effector position [x, y, z] in the base frame {0}, in metres.

        Returns
        -------
        list of dict, one per obstacle:
            "distance" : float, distance from the end effector to the obstacle's surface (m)
            "direction" : np.ndarray, shape (3,), unit vector from the obstacle toward the end effector
        """
        obstacle_data = []
        for obst in self.obstacles:
            dist, direction = self.get_ee_to_obstacle_vector(p4, obst)
            obstacle_data.append({
                'distance': dist,
                'direction': direction
            })
        return obstacle_data
