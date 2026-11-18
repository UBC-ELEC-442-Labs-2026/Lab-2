import numpy as np
import numpy.linalg as npla

class Numeric_IK_helper():
    """QArm utilities, such as Forward/Inverse Kinematics, Differential Kinematics etc."""
    #region: QArm Parameters

    L_1 = 0.1400
    L_2 = 0.3500
    L_3 = 0.0500
    L_4 = 0.2500
    L_5 = 0.1500
    BETA = np.arctan(L_3/L_2)

    LAMBDA_1 = L_1
    LAMBDA_2 = np.sqrt(L_2**2 + L_3**2)
    LAMBDA_3 = L_4 + L_5
    #endregion

    def forward_kinematics(self, phi):
        """
        Forward kinematics of the QArm.

        Parameters
        ----------
        phi : np.ndarray, shape (4,)
            Joint angles [base, shoulder, elbow, wrist], in radians.

        Returns
        -------
        p4 : np.ndarray, shape (3,)
            End-effector position [x, y, z] in the base frame {0}, in metres.
        R04 : np.ndarray, shape (3, 3)
            Rotation from the end-effector frame {4} to the base frame {0}.
        """

        # From phi space to theta space
        theta = phi.copy()
        theta[0] = phi[0]
        theta[1] = phi[1] + self.BETA - np.pi/2
        theta[2] = phi[2] - self.BETA
        theta[3] = phi[3]

        # Transformation matrices for all frames:

        # T{i-1}{i} = quanser_arm_DH(  a, alpha,  d,     theta )
        T01 = self.quanser_arm_DH(            0, -np.pi/2, self.LAMBDA_1,  theta[0] )
        T12 = self.quanser_arm_DH( self.LAMBDA_2,        0,            0,  theta[1] )
        T23 = self.quanser_arm_DH(            0, -np.pi/2,            0,  theta[2] )
        T34 = self.quanser_arm_DH(            0,        0, self.LAMBDA_3,  theta[3] )

        T02 = T01@T12
        T03 = T02@T23
        T04 = T03@T34

        # Position of end-effector Transformation

        # Extract the Position vector
        # p1   = T01(1:3,4);
        # p2   = T02(1:3,4);
        # p3   = T03(1:3,4);
        p4   = T04[0:3,3]

        # Extract the Rotation matrix
        # R01 = T01(1:3,1:3);
        # R02 = T02(1:3,1:3);
        # R03 = T03(1:3,1:3);
        R04 = T04[0:3,0:3]

        return p4, R04

    def check_joint_limits(self, phi):
        """
        Check that every joint is within its limits.

        Parameters
        ----------
        phi : np.ndarray, shape (4,)
            Joint angles [base, shoulder, elbow, wrist], in radians.

        Returns
        -------
        bool
            True if every joint is within its limits, False otherwise.
        """
        flag = True
        if phi[0] > 170*np.pi/180 or phi[0] < -170*np.pi/180 \
            or phi[1] > 85*np.pi/180 or phi[1] < -85*np.pi/180 \
            or phi[2] > 75*np.pi/180 or phi[2] < -95*np.pi/180 \
            or phi[3] > 160*np.pi/180 or phi[3] < -160*np.pi/180:

                flag = False

        return flag

    def quanser_arm_DH(self, a, alpha, d, theta):

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

    def differential_kinematics(self, phi):
        """Implements the jacobian matrix using theta for the QArm Mini

        Parameters:

        theta (): Alternate joint angle vector 4 x 1

        Return:
            - **J** *()* - Best solution depending on theta_prev
            - **c** *()* - condition number of the jacobian matrix
            - **r** *()* - rank of the jacobian matrix
            - **J_inv** *()* - inverse of the jacobian matrix
        """
        # jacobian for the qarm mini
        theta = phi.copy()
        theta[0] = phi[0]
        theta[1] = phi[1] + self.BETA - np.pi/2
        theta[2] = phi[2] - self.BETA
        theta[3] = phi[3]
        J = np.identity(4)

        J[0,0] = -self.LAMBDA_2*np.sin(theta[0])*np.cos(theta[1]) + self.LAMBDA_3*np.sin(theta[0])*np.sin(theta[1] + theta[2])
        J[0,1] = -self.LAMBDA_2*np.cos(theta[0])*np.sin(theta[1]) - self.LAMBDA_3*np.cos(theta[0])*np.cos(theta[1] + theta[2])
        J[0,2] = -self.LAMBDA_3*np.cos(theta[0])*np.cos(theta[1] + theta[2])
        J[0,3] = 0

        J[1,0] = self.LAMBDA_2*np.cos(theta[0])*np.cos(theta[1]) - self.LAMBDA_3*np.cos(theta[0])*np.sin(theta[1]+theta[2])
        J[1,1] = -self.LAMBDA_2*np.sin(theta[0])*np.sin(theta[1]) - self.LAMBDA_3*np.sin(theta[0])*np.cos(theta[1]+theta[2])
        J[1,2] = -self.LAMBDA_3*np.sin(theta[0])*np.cos(theta[1] + theta[2])
        J[1,3] = 0

        J[2,0] = 0
        J[2,1] = -self.LAMBDA_2*np.cos(theta[1]) + self.LAMBDA_3*np.sin(theta[1] + theta[2])
        J[2,2] = self.LAMBDA_3*np.sin(theta[1] + theta[2])
        J[2,3] = 0

        J[3,0] = 0
        J[3,1] = 0
        J[3,2] = 0
        J[3,3] = 1

        c = npla.cond(J) #condition number for the Jacobian
        r = npla.matrix_rank(J) # rank number for the jacobian
        J_inv = npla.inv(J) # inverse of the jacobian matrix

        return J, c, r, J_inv

    def Jacobian(self, phi):
        """
        The 4x4 analytical Jacobian of the QArm.

        Parameters
        ----------
        phi : np.ndarray, shape (4,)
            Joint angles [base, shoulder, elbow, wrist], in radians.

        Returns
        -------
        np.ndarray, shape (4, 4)
            Rows 1 to 3 map joint velocities to end-effector velocity [x, y, z];
            row 4 is the wrist.
        """
        return self.differential_kinematics(phi)[0]

    def Inv_Jacobian(self, phi):
        """
        The inverse of the 4x4 analytical Jacobian of the QArm.

        Parameters
        ----------
        phi : np.ndarray, shape (4,)
            Joint angles [base, shoulder, elbow, wrist], in radians.

        Returns
        -------
        np.ndarray, shape (4, 4)
            Inverse of Jacobian(phi). Very large near a singularity.
        """
        return self.differential_kinematics(phi)[3]