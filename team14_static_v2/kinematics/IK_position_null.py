import numpy as np
from .calculateFK import FK
from .calcJacobian import calcJacobian
from .calcAngDiff import calcAngDiff


class IK:
    lower = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    upper = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
    center = (lower + upper) / 2
    fk = FK()

    def __init__(self, linear_tol=1e-4, angular_tol=1e-3,
                 max_steps=1000, min_step_size=1e-5):
        self.linear_tol = linear_tol
        self.angular_tol = angular_tol
        self.max_steps = max_steps
        self.min_step_size = min_step_size
        self.last_iterations = 0
        self.last_termination = 'not_started'

    @staticmethod
    def displacement_and_axis(target, current):
        return (np.asarray(target[:3, 3] - current[:3, 3]),
                calcAngDiff(target[:3, :3], current[:3, :3]))

    @staticmethod
    def distance_and_angle(G, H):
        distance = float(np.linalg.norm(G[:3, 3] - H[:3, 3]))
        relative = G[:3, :3] @ H[:3, :3].T
        angle = float(np.arccos(np.clip((np.trace(relative) - 1) / 2, -1., 1.)))
        return distance, angle

    def is_valid_solution(self, q, target):
        q = np.asarray(q, dtype=float).reshape(7)
        if not np.isfinite(target).all():
            return False, 'Nonfinite target pose'
        if not np.isfinite(q).all():
            return False, 'Nonfinite joint angles'
        if np.any(q < self.lower) or np.any(q > self.upper):
            return False, 'Joint limit violation'
        distance, angle = self.distance_and_angle(target, self.fk.forward(q)[1])
        if distance >= self.linear_tol:
            return False, 'Position tolerance not met'
        if angle >= self.angular_tol:
            return False, 'Orientation tolerance not met'
        return True, 'Solution found within limits and pose tolerances'

    @staticmethod
    def end_effector_task(q, target, method='J_pseudo'):
        current = IK.fk.forward(q)[1]
        displacement, axis = IK.displacement_and_axis(target, current)
        error = np.concatenate((displacement, axis))
        J = calcJacobian(q)
        if method == 'J_pseudo':
            return np.linalg.pinv(J) @ error
        if method == 'J_trans':
            return J.T @ error
        raise ValueError('method must be J_pseudo or J_trans')

    @staticmethod
    def joint_centering_task(q, rate=5e-1):
        offset = 2 * (q - IK.center) / (IK.upper - IK.lower)
        return -rate * offset

    def inverse(self, target, seed, alpha=0.5, method='J_pseudo'):
        if method not in ('J_pseudo', 'J_trans') or alpha <= 0:
            raise ValueError('Invalid method or alpha')
        q = np.asarray(seed, dtype=float).reshape(7).copy()
        rollout = [q.copy()]
        self.last_iterations = 0
        self.last_termination = 'max_steps'
        for _ in range(self.max_steps):
            # During a timed match a toleranced pose is already an acceptable
            # endpoint; continuing just to improve the secondary task costs time.
            if self.is_valid_solution(q, target)[0]:
                self.last_termination = 'pose_reached'
                break
            J = calcJacobian(q)
            J_pinv = np.linalg.pinv(J)
            error = np.concatenate(self.displacement_and_axis(target, self.fk.forward(q)[1]))
            primary = (J_pinv if method == 'J_pseudo' else J.T) @ error
            secondary = self.joint_centering_task(q)
            dq = primary + (np.eye(7) - J_pinv @ J) @ secondary
            if not np.isfinite(dq).all():
                self.last_termination = 'nonfinite_step'
                break
            if np.linalg.norm(dq) < self.min_step_size:
                self.last_termination = 'small_step'
                break
            # Bounds are enforced on finite updates; FK validation remains necessary.
            new_q = np.clip(q + alpha * dq, self.lower, self.upper)
            self.last_iterations += 1
            rollout.append(new_q.copy())
            if np.linalg.norm(new_q - q) < self.min_step_size * alpha:
                q = new_q
                self.last_termination = 'bounded_stagnation'
                break
            q = new_q
        success, message = self.is_valid_solution(q, target)
        return q, rollout, success, message


if __name__ == '__main__':
    seed = np.array([0, 0, 0, -np.pi / 2, 0, np.pi / 2, np.pi / 4])
    target = np.array([[0, -1, 0, -.2], [-1, 0, 0, 0],
                       [0, 0, -1, .5], [0, 0, 0, 1.]])
    for method in ('J_pseudo', 'J_trans'):
        ik = IK()
        q, rollout, success, message = ik.inverse(target, seed, method=method)
        print(method, success, message, 'iterations:', ik.last_iterations)
        print('q:', q, 'errors:', ik.distance_and_angle(target, ik.fk.forward(q)[1]))
