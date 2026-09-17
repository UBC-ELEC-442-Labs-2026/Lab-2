import numpy as np

def geometric_inv_kin(p, wrist, phi_prev):
    """
    INPUTS:
        p: desired end-effector position vector expressed in base frame {0}
        wrist: desired wrist rotation angle gamma in radians
        phi_prev: current/last joint positions

    OUTPUTS:
        phiOptimal : Best valid solution depending on phi_prev
        phi: All four Inverse Kinematics solutions as a 4x4 matrix. Each col is a solution.
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

    phi = np.mod(phi + np.pi, 2*np.pi) - np.pi


    # == start student section ==

    #TODO: implement a way to find the optimal solution given phi_prev
    phiOptimal = phi[:,0]

    # == end student section ==


    return phi, phiOptimal


if __name__ == "__main__":
    print("Test your function here")