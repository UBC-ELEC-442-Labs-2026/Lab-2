from pal.products.qarm import QArm
from hal.products.qarm import QArmUtilities
import time
import numpy as np
from scipy.interpolate import CubicSpline

# So the goal here is to implement obstacle avoidance by using potential fields and Forces

obstacle_locations_xyz = np.array([
    np.array([0.6, 0.0, 0.3]), # This one is the actual box, the others are imaginary
    np.array([0.5, 0.0, 0.3]),
    np.array([0.6, -0.1, 0.3])
])

P_start = np.array([0.6, -0.3, 0.3])
P_end = np.array([0.6, 0.3, 0.3])
P_home = np.array([0.45, 0, 0.49])

def attractive_potential(P_arm, P_interest, max_dist, force_scaling):
    vec = P_interest - P_arm # Vector points towards target
    dist = np.linalg.norm(vec)
    #return force_scaling * (vec/dist) # capped force
    if dist > max_dist:
        return force_scaling * (vec/dist) # capped force
    else:
        return force_scaling * (vec) # 3D force in direction

def repulsive_potential(P_arm, P_interest, max_dist, force_scaling):
    vec = P_arm - P_interest # Vector points away from target
    dist = np.linalg.norm(vec)
    if dist > max_dist:
        return np.array([0, 0, 0]) # No force at distance
    else:
        scalar_force = force_scaling * (1.0/dist - 1.0/max_dist) * (1.0 / dist**2)
        return scalar_force * (vec / dist)



with QArm(hardware=0, readMode=0) as myArm:
    myArmUtilities = QArmUtilities()

    # Move to start point
    allPhi, q_next = myArmUtilities.inverse_kinematics(P_start, 0, myArm.measJointPosition[0:4])
    print("Moving to start position...")
    myArm.read_write_std(phiCMD=q_next, gprCMD=1, baseLED=0)
    time.sleep(2) # Just wait for a second
    location, rotation = myArmUtilities.forward_kinematics(q_next)

    last_time = time.time()

    while np.linalg.norm(P_end - location) > 0.01:
        print(f"Locations: {location}, {np.linalg.norm(P_end - location)}")
        # Calculate forces
        # This should be done for various points along the arm, each with their own Jacobian
        # Let's start with the end effector for now though to keep it simple
        F = np.array([0.0, 0.0, 0.0])
        myArm.read_std()
        

        # Repulsive forces from obstacles (treated as points)
        for ob in obstacle_locations_xyz:
            F += repulsive_potential(location, ob, 0.1, 1)

        #Attractive force from end point
        F += attractive_potential(location, P_end, 0.01, 1)

        _, _, _, J_inv = myArmUtilities.differential_kinematics(myArm.measJointPosition[0:4])
        q_dot = J_inv @ np.append(F, 0)
        max_joint_speed = 1.0  # rad/s max limit
        q_dot = np.clip(q_dot, -max_joint_speed, max_joint_speed)

        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        q_next = myArm.measJointPosition[0:4] + q_dot * dt
        myArm.read_write_std(phiCMD=q_next, gprCMD=1, baseLED=0)

        location, rotation = myArmUtilities.forward_kinematics(myArm.measJointPosition[0:4])

        
        time.sleep(0.02)

    # Return home
    time.sleep(2)
    allPhi, q_next = myArmUtilities.inverse_kinematics(P_home, 0, myArm.measJointPosition[0:4])
    print("Moving to start position...")
    myArm.read_write_std(phiCMD=q_next, gprCMD=1, baseLED=0)
    time.sleep(1) # Just wait for a second

    myArm.terminate()
