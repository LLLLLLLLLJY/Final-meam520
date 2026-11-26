
import numpy as np
from math import pi


class FK():

    def __init__(self):

        # DH parameters:
        # [a_i, alpha_i, d_i, theta_offset]
        self.dh_params = np.array([
            [0.0,      -pi/2, 0.333,  0.0],
            [0.0,       pi/2, 0.0,    0.0],
            [0.0825,    pi/2, 0.316,  0.0],
            [-0.0825,  -pi/2, 0.0,    0.0],
            [0.0,       pi/2, 0.384,  0.0],
            [0.088,     pi/2, 0.0,    0.0],
            [0.0,       0.0,  0.210, -pi/4]
        ])

        # Canonical joint positions relative to the corresponding
        # intermediate DH frames.
        self.joint_offsets = np.array([
            [0.0, 0.0,  0.141, 1.0],   # Joint 1
            [0.0, 0.0,  0.0,   1.0],   # Joint 2
            [0.0, 0.0,  0.195, 1.0],   # Joint 3
            [0.0, 0.0,  0.0,   1.0],   # Joint 4
            [0.0, 0.0,  0.125, 1.0],   # Joint 5
            [0.0, 0.0, -0.015, 1.0],   # Joint 6
            [0.0, 0.0,  0.051, 1.0],   # Joint 7
            [0.0, 0.0,  0.0,   1.0]    # End effector
        ])

    def dh_transform(self, a, alpha, d, theta):

        c = np.cos(theta)
        s = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        return np.array([
            [c, -s * ca,  s * sa, a * c],
            [s,  c * ca, -c * sa, a * s],
            [0,      sa,      ca,     d],
            [0,       0,       0,     1]
        ])

    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions - 8 x 3 matrix, where each row corresponds to
                         a rotational joint or the end effector.
                         Each row contains [x, y, z] coordinates in
                         the world frame.

        T0e - 4 x 4 homogeneous transformation matrix representing
              the end-effector frame in the world frame.
        """

        q = np.asarray(q).reshape(7)

        jointPositions = np.zeros((8, 3))

        # Joint 1 has a fixed canonical position in the world frame.
        jointPositions[0] = self.joint_offsets[0, :3]

        # Start from the world frame.
        T = np.eye(4)

        # Apply the seven DH transformations.
        for i in range(7):

            a = self.dh_params[i, 0]
            alpha = self.dh_params[i, 1]
            d = self.dh_params[i, 2]
            theta_offset = self.dh_params[i, 3]

            A = self.dh_transform(
                a,
                alpha,
                d,
                q[i] + theta_offset
            )

            T = T @ A

            # Canonical location of the next joint / end effector.
            p = T @ self.joint_offsets[i + 1]

            jointPositions[i + 1] = p[:3]

        T0e = T

        return jointPositions, T0e

    # This code is for Lab 2, you can ignore it for Lab 1.
    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list - 3x7 np array of unit vectors describing
                                the axis of rotation for each joint in
                                the world frame
        """

        A = self.compute_Ai(q)

        axes = np.zeros((3, 7))
        T = np.eye(4)

        for i in range(7):
            axes[:, i] = T[:3, 2]
            T = T @ A[i]

        return axes

    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles

        OUTPUTS:
        Ai - list of 4x4 homogeneous transformations describing the FK.
        """

        q = np.asarray(q, dtype=float).reshape(7)

        A = []

        for i in range(7):
            a, alpha, d, theta_offset = self.dh_params[i]
            theta = q[i] + theta_offset

            Ai = self.dh_transform(a, alpha, d, theta)
            A.append(Ai)

        return A


if __name__ == "__main__":

    fk = FK()

    # Configuration shown in Figure 2 of the handout.
    q = np.array([
        0,
        0,
        0,
        0,
        0,
        0,
        0
    ])

    joint_positions, T0e = fk.forward(q)

    print("Joint Positions:\n", joint_positions)
    print("End Effector Pose:\n", T0e)

