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
        """The obstacle as [Q1, Q2, R]."""
        return np.array([self.Q1, self.Q2, self.R])
    


    
class Potential_Fields_helper:
    """
    Distances and directions from the end effector to the obstacles, for the potential field.
    """

    def __init__(self, obstacles):
        """obstacles: list of Cylinder_Obstacle."""
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
        p4 = np.asarray(p4, dtype=float)
        Q1 = np.asarray(obst.Q1, dtype=float)

        # The cylinder's axis: unit direction k and length
        axis = np.asarray(obst.Q2, dtype=float) - Q1
        length = np.linalg.norm(axis)
        k = axis / length

        # Split p4 into a height along the axis and a sideways offset from it
        rel = p4 - Q1
        h = np.dot(rel, k)
        radial = rel - h * k
        rho = np.linalg.norm(radial)

        # Unit vector pointing sideways from the axis toward p4
        if rho > 1e-9:
            radial_dir = radial / rho
        else:
            # p4 is exactly on the axis: any sideways direction will do
            if abs(k[2]) < 0.9:
                radial_dir = np.cross(k, [0.0, 0.0, 1.0])
            else:
                radial_dir = np.cross(k, [1.0, 0.0, 0.0])
            radial_dir = radial_dir / np.linalg.norm(radial_dir)

        # Closest point on the solid cylinder: clamp the height to [0, length] and the radius to R
        closest = Q1 + np.clip(h, 0.0, length) * k + min(rho, obst.R) * radial_dir
        vector_to_ee = p4 - closest
        distance_to_skin = np.linalg.norm(vector_to_ee)

        if distance_to_skin < 1e-9:
            # Inside the cylinder: push out through the nearest face (the side wall or the top)
            if (obst.R - rho) < (length - h):
                return 0.0, radial_dir
            return 0.0, k

        return distance_to_skin, vector_to_ee / distance_to_skin
    

    
    def check_pose(self, phi: np.ndarray, target_pos: np.ndarray, name: str) -> None:
        """
        Stop the program with a message if phi doesn't put the tip at target_pos (the inverse
        kinematics failed), or if the arm would collide there. Use it on the start and the goal.
        """
        # Same check as the RRT exercise
        from helpers.RRT_helpers import RRT_Helpers
        RRT_Helpers(self.obstacles).check_pose(phi, target_pos, name)

    def plot_path(self, path: list[np.ndarray]) -> None:
        """
        3D plot of the obstacles, the arm at the start (green) and the end (red) of the path,
        and the line the fingertip followed (orange). Close the window to carry on.
        """
        # Same drawing as the RRT exercise
        from helpers.RRT_helpers import RRT_Helpers
        RRT_Helpers(self.obstacles).plot_path(path, title="Path taken (close this window to finish)")

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
