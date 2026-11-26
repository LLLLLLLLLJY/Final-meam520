
import numpy as np
from .calculateFK import FK


def calcJacobian(q_in):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """

    J = np.zeros((6, 7))

    fk = FK()
    q = np.asarray(q_in, dtype=float).reshape(7)

    # Forward kinematics
    _, T0e = fk.forward(q)
    oe = T0e[:3, 3]

    # Joint rotation axes in the world frame
    axes = fk.get_axis_of_rotation(q)

    # DH transformations
    A = fk.compute_Ai(q)
    T = np.eye(4)

    for i in range(7):
        z = axes[:, i]
        o = T[:3, 3]

        # Linear velocity Jacobian
        J[:3, i] = np.cross(z, oe - o)

        # Angular velocity Jacobian
        J[3:, i] = z

        # Update transformation for the next joint
        T = T @ A[i]

    return J


if __name__ == "__main__":
    q = np.array([0, 0, 0, -np.pi / 2, 0, np.pi / 2, np.pi / 4])
    print(np.round(calcJacobian(q), 3))

