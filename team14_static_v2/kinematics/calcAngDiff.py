
import numpy as np


def calcAngDiff(R_des, R_curr):
    """
    Helper function for the End Effector Orientation Task.
    Computes the axis of rotation from the current orientation
    to the target orientation.

    INPUTS:
    R_des - 3x3 desired orientation
    R_curr - 3x3 current orientation

    OUTPUTS:
    omega - 3-element rotation vector in the world frame.
    Its magnitude is sin(angle).
    """

    Rrel = R_curr.T @ R_des

    skew = 0.5 * (Rrel - Rrel.T)

    u = np.array([
        skew[2, 1],
        skew[0, 2],
        skew[1, 0]
    ])

    omega = R_curr @ u

    return omega

