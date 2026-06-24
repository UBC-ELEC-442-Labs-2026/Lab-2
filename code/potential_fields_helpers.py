from pal.products.qarm import QArm
from hal.products.qarm import QArmUtilities
import numpy as np

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
        Calculates the minimum distance and direction vector from the End-Effector (p4)
        to a cylindrical obstacle.
        
        Parameters:
        - p4: np.array([x, y, z]) of the end-effector
        - obst: Cylinder_Obstacle object containing Q1, Q2, and R
        
        Returns:
        - distance_to_skin: Float, shortest distance from EE to the cylinder surface
        - direction_vector: np.array([x, y, z]), normalized vector pointing from the 
                            obstacle surface directly toward p4 in {0} coordinates
        """
        # Vector along the cylinder's centerline
        v = obst.Q2 - obst.Q1
        # Vector from cylinder start to the End-Effector
        w = p4 - obst.Q1
        
        # Project w onto v to find the closest point on the infinite line
        projection = np.dot(w, v) / np.dot(v, v)
        
        # Clamp the projection to the finite segment bounds [0, 1] 
        # This correctly handles the flat caps/ends of the cylinder
        t = np.clip(projection, 0.0, 1.0)
        
        # Closest point on the cylinder's centerline axis
        Q_closest = obst.Q1 + t * v
        
        # Vector from the centerline to the End-Effector
        vector_to_ee = p4 - Q_closest
        distance_to_center = np.linalg.norm(vector_to_ee)
        
        # Handle the edge case where the EE is exactly on the centerline axis
        if distance_to_center < 1e-6:
            # Provide a default fallback direction to prevent division by zero
            return 0.0, np.array([1.0, 0.0, 0.0])
            
        # Subtract the cylinder's radius to get the distance to the outer skin
        distance_to_skin = distance_to_center - obst.R
        
        # Normalized direction vector pointing away from the obstacle toward the EE
        direction_vector = vector_to_ee / distance_to_center
        
        return distance_to_skin, direction_vector
    

    
    def get_all_obstacle_distances(self, p4):
        """
        Queries all environment obstacles relative to the current End-Effector position.
        
        Returns:
        A list of dictionaries, where each entry contains:
        - 'distance': Distance to the skin of that obstacle (float)
        - 'direction': Normalized direction vector pointing toward safety (np.array)
        """
        obstacle_data = []
        for obst in self.obstacles:
            dist, direction = self.get_ee_to_obstacle_vector(p4, obst)
            obstacle_data.append({
                'distance': dist,
                'direction': direction
            })
        return obstacle_data