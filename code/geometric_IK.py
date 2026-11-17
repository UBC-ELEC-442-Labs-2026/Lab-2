import numpy as np

def geometric_inv_kin(p: np.ndarray, wrist: float, phi_prev: np.ndarray):
    """
    Inverse kinematics of the QArm by the geometric method.

    Parameters
    ----------
    p : np.ndarray, shape (3,)
        Desired end-effector position [x, y, z] in the base frame {0}, in metres.
    wrist : float
        Desired wrist angle, in radians.
    phi_prev : np.ndarray, shape (4,)
        Current joint angles [base, shoulder, elbow, wrist], in radians.

    Returns
    -------
    phi_candidates : np.ndarray, shape (4, 4), or "Fail"
        The four mathematical solutions (candidates), one per column. Some may
        break the joint limits. "Fail" if the target is out of reach.
    phi_best : np.ndarray, shape (4,), or "Fail"
        The valid candidate closest to phi_prev, or "Fail" if no candidate is valid.
    """

    L_1 = 0.1400
    L_2 = 0.3500
    L_3 = 0.0500
    L_4 = 0.2500
    L_5 = 0.1500
    BETA = np.arctan(L_3/L_2)

    LAMBDA_1 = L_1
    LAMBDA_2 = np.sqrt(L_2**2 + L_3**2)
    LAMBDA_3 = L_4 + L_5

    # Joint limits in radians, in order [base, shoulder, elbow, wrist] (Lab 2 PDF, joint limits table)
    JOINT_MIN = np.radians([-170, -85, -95, -160])
    JOINT_MAX = None  # TODO: the maximum joint angles

    # Initialization
    theta   = np.zeros((4, 4), dtype=np.float64)
    phi = np.zeros((4, 4), dtype=np.float64)

    # Equations:
    # LAMBDA_2 cos(theta2) + (-LAMBDA_3) sin(theta2 + theta3) = sqrt(x^2 + y^2)
    #   A     cos( 2    ) +     C      sin(   2   +    3  ) =    D

    # LAMBDA_2 sin(theta2) - (-LAMBDA_3) cos(theta2 + theta3) = LAMBDA_1 - z
    #   A     sin( 2    ) -     C      cos(   2   +    3  ) =    H

    # Solution:
    def inv_kin_setup(p):
        A = LAMBDA_2
        C = -LAMBDA_3
        H =  LAMBDA_1 - p[2]
        D1 = -np.sqrt(p[0]**2 + p[1]**2)
        D2 =  np.sqrt(p[0]**2 + p[1]**2)
        F = (D1**2 + H**2 - A**2 - C**2)/(2*A)
        return A, C, H, D1, D2, F

    def solve_case_C_j2(j3, A, C, D, H):
        M = A + C*np.sin(j3)
        N = -C*np.cos(j3)
        cos_term = (D*M + H*N)/(M**2 + N**2)
        sin_term = (H - N*cos_term)/(M)
        j2 = np.arctan2(sin_term, cos_term)
        return j2

    A, C, H, D1, D2, F = inv_kin_setup(p)

    # == start student section ==

    # Out of reach: the square roots below would be of a negative number
    if C**2 - F**2 < 0:
        print("geometric_inv_kin: target is out of reach")
        # TODO: return the appropriate result for both outputs
        pass

    # ==  end student section  ==

    # Joint 3 solution:
    theta[2,0] = 2*np.arctan2( C + np.sqrt(C**2 - F**2) , F )
    theta[2,1] = 2*np.arctan2( C - np.sqrt(C**2 - F**2) , F )
    theta[2,2] = 2*np.arctan2( C + np.sqrt(C**2 - F**2) , F )
    theta[2,3] = 2*np.arctan2( C - np.sqrt(C**2 - F**2) , F )

    # Joint 2 solution:
    theta[1,0] = solve_case_C_j2(theta[2,0], A, C, D1, H)
    theta[1,1] = solve_case_C_j2(theta[2,1], A, C, D1, H)
    theta[1,2] = solve_case_C_j2(theta[2,2], A, C, D2, H)
    theta[1,3] = solve_case_C_j2(theta[2,3], A, C, D2, H)

    # Joint 1 solution:
    theta[0,0] = np.arctan2( p[1]/( LAMBDA_2 * np.cos( theta[1,0] ) - LAMBDA_3 * np.sin( theta[1,0] + theta[2,0] ) ) ,
                                p[0]/( LAMBDA_2 * np.cos( theta[1,0] ) - LAMBDA_3 * np.sin( theta[1,0] + theta[2,0] )  ) )
    theta[0,1] = np.arctan2( p[1]/( LAMBDA_2 * np.cos( theta[1,1] ) - LAMBDA_3 * np.sin( theta[1,1] + theta[2,1] ) ) ,
                                p[0]/( LAMBDA_2 * np.cos( theta[1,1] ) - LAMBDA_3 * np.sin( theta[1,1] + theta[2,1] )  ) )
    theta[0,2] = np.arctan2( p[1]/( LAMBDA_2 * np.cos( theta[1,2] ) - LAMBDA_3 * np.sin( theta[1,2] + theta[2,2] ) ) ,
                                p[0]/( LAMBDA_2 * np.cos( theta[1,2] ) - LAMBDA_3 * np.sin( theta[1,2] + theta[2,2] )  ) )
    theta[0,3] = np.arctan2( p[1]/( LAMBDA_2 * np.cos( theta[1,3] ) - LAMBDA_3 * np.sin( theta[1,3] + theta[2,3] ) ) ,
                                p[0]/( LAMBDA_2 * np.cos( theta[1,3] ) - LAMBDA_3 * np.sin( theta[1,3] + theta[2,3] )  ) )

    # Remap theta back to phi
    phi[0,:] = theta[0,:]
    phi[1,:] = theta[1,:] - BETA + np.pi/2
    phi[2,:] = theta[2,:] + BETA
    phi[3,:] = wrist*np.ones((4))

    phi_candidates = np.mod(phi + np.pi, 2*np.pi) - np.pi   # wrap every angle to [-pi, pi)


    # == start student section ==

    # Note two additional TODOs above

    # Set phi_best to the column of phi_candidates that
    #   1. is valid (physically achievable), and
    #   2. of those, is the closest (np.linalg.norm).
    # If those conditions cannot be met, set phi_best = "Fail".

    # tip: Use the Python Reference, section 1.4 of the Lab PDF

    # hint: chained comparisons, (a < b < c), don't work on NumPy arrays,
    # compare the sides separately and combine the results

    # Placeholder: Sets phi_best to the first column of phi_candidates
    phi_best = phi_candidates[:, 0]

    # ==  end student section  ==


    return phi_candidates, phi_best


if __name__ == "__main__":
    # test cases
    test_cases = [
        # (description,             p,                  wrist,  phi_prev)
        ("Normal",                 [0.3, 0.4, 0.35],    0.87,  [0, 0, 0, 0]),
        ("Invalid wrist",          [0.3, 0.4, 0.35],    3.0,   [0, 0, 0, 0]),
        ("Negative position",      [-0.2, -0.2, 0.6],  -2.62,  [0, 0, 0, 0]),
        ("Out of workspace",       [0.75, 0.3, 0.5],    0,     [0, 0, 0, 0]),
        ("Singularity at end",     [0, 0, 0.75],        0,     [0, 0, 0, 0]),
        ("Singularity near start", [0.2, 0.2, 0.3],     0,     [1.57, -0.82, -0.17, 0]),
        ("Singularity in between", [0.01, -0.455, 0.5], 0,     [np.pi/2, 0, -0.0254, 0]),
    ]

    # Printing shows 2 decimal places
    np.set_printoptions(precision=2, suppress=True)

    for description, p, wrist, phi_prev in test_cases:
        print(f"--- {description}: p = {p}, wrist = {wrist}, phi_prev = {np.array(phi_prev, dtype=float)}")
        phi_candidates, phi_best = geometric_inv_kin(np.array(p), wrist, np.array(phi_prev))
        print("phi_candidates:")
        print(phi_candidates)
        print("phi_best:")
        print(phi_best)
        print()