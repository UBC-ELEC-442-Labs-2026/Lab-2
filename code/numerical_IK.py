import sys
import os

import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from helpers.numerical_IK_helpers import Numeric_IK_helper

# End of imports

def numeric_inv_kin(p: np.ndarray, wrist: float, phi_prev: np.ndarray):
    """
    Inverse kinematics of the QArm by the numerical (Newton-Raphson) method.

    Starting from phi_prev, repeatedly steps the joint angles toward the target
    using the inverse Jacobian, until the end effector reaches p.

    Parameters
    ----------
    p : np.ndarray, shape (3,)
        Desired end-effector position [x, y, z] in the base frame {0}, in metres.
    wrist : float
        Desired wrist angle, in radians.
    phi_prev : np.ndarray, shape (4,)
        Current joint angles [base, shoulder, elbow, wrist], in radians.
        The solver starts from here.

    Returns
    -------
    phi_new : np.ndarray, shape (4,), or "Fail"
        Joint angles that reach p, or "Fail" if a step breaks the joint limits
        or the solver hasn't converged after attempt_limit steps.
    """

    num_ik_helper = Numeric_IK_helper()

    # Helper functions (phi is a vector of 4 joint angles):
    #   J_inv = num_ik_helper.Inv_Jacobian(phi)        inverse of the 4x4 analytical Jacobian
    #   p4, _ = num_ik_helper.forward_kinematics(phi)  p4 is the end-effector position [x, y, z]
    #   num_ik_helper.check_joint_limits(phi)          True if every joint is within its limits

    # Solver parameters
    success_thresh = 0.0001     # stop once the error is smaller than this
    step_size_scaling = 0.01    # alpha in the update equation
    attempt_limit = 2000        # maximum number of update steps

    phi_new = np.array(phi_prev, dtype=float)   # the solver's starting point: a copy of phi_prev

    # == start student section ==

    # Repeat the update equation from the Numerical Method section until the error is
    # smaller than success_thresh. Set phi_new = "Fail" if a step breaks the joint
    # limits, or if attempt_limit steps go by without converging.

    # tip: use the three helpers above

    # ==  end student section  ==

    return phi_new


if __name__ == "__main__":
    # The Exercise 2 test cases from the Lab 2 PDF. Run this file and compare the
    # printed results with the table. You don't need to change anything below.
    test_cases = [
        # (description,             p,                   wrist, phi_prev)
        ("Normal",                 [0.3, 0.4, 0.35],    0.87,  [0, 0, 0, 0]),
        ("Invalid wrist",          [0.3, 0.4, 0.35],    3.0,   [0, 0, 0, 0]),
        ("Negative position",      [-0.2, -0.2, 0.6],  -2.62,  [0, 0, 0, 0]),
        ("Out of workspace",       [0.75, 0.3, 0.5],    0,     [0, 0, 0, 0]),
        ("Singularity at end",     [0, 0, 0.75],        0,     [0, 0, 0, 0]),
        ("Singularity near start", [0.2, 0.2, 0.3],     0,     [1.57, -0.82, -0.17, 0]),
        ("Singularity in between", [0.01, -0.455, 0.5], 0,     [np.pi/2, 0, -0.0254, 0]),
    ]

    # Printing shows 2 decimal places; the function itself never rounds.
    np.set_printoptions(precision=2, suppress=True)

    for description, p, wrist, phi_prev in test_cases:
        print(f"--- {description}: p = {p}, wrist = {wrist}, phi_prev = {np.array(phi_prev, dtype=float)}")
        phi_new = numeric_inv_kin(np.array(p), wrist, np.array(phi_prev))
        print("phi_new:", phi_new)
        print()