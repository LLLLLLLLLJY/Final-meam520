"""Static-block observation and single-grasp checks for both team colors."""
import numpy as np
from kinematics.calculateFK import FK
from kinematics.IK_position_null import IK


VIEW = np.array([-0.1036, -0.0450, -0.2051, -1.7048, -0.0092, 1.6607, 0.4778])


def select_upper_cube(blocks, team="red"):
    candidates = []
    for name, pose in blocks.items():
        x, y, z = pose[:3, 3]
        if ("static" in name.lower() and "rect" not in name.lower()
                and np.isfinite(pose).all()
                and abs(x - 0.562) < 0.16
                and abs(y - (-0.169 if team == "red" else 0.169)) < 0.16
                and 0.255 < z < 0.305):
            candidates.append((name, pose))
    if not candidates:
        raise RuntimeError("Upper static cube not visible on the " + team + " team's source platform.")
    return max(candidates, key=lambda item: item[1][2, 3])


def grasp_rotation(block):
    axis = block[:3, np.argmin(np.abs(block[2, :3]))]
    yaw = (np.arctan2(axis[1], axis[0]) + np.pi / 4) % (np.pi / 2) - np.pi / 4
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, s, 0], [s, -c, 0], [0, 0, -1]])


def solve(ik, seed, rotation, position):
    target = np.eye(4)
    target[:3, :3] = rotation
    target[:3, 3] = position
    q, _, ok, message = ik.inverse(target, seed, alpha=0.5)
    if not ok:
        raise RuntimeError("IK: " + str(message))
    return q


def move(arm, q):
    arm.safe_move_to_position(q)
    measured = np.asarray(arm.get_positions()).reshape(7)
    if not np.isfinite(measured).all() or np.max(np.abs(measured - q)) > 0.02:
        raise RuntimeError("Commanded pose was not reached.")
    return measured


def width_mm(arm):
    fingers = np.asarray(arm.get_gripper_state().get("position", []))
    return float(fingers.sum() * 1000) if fingers.size == 2 else float("nan")


def observe(arm, detector, fk, rospy, clock):
    _, base_ee = fk.forward(arm.get_positions())
    base_camera = base_ee @ detector.get_H_ee_camera()
    blocks = {}
    start = clock()
    while not rospy.is_shutdown() and clock() - start < 3.0:
        for name, pose in detector.get_detections():
            if "static" in name.lower():
                blocks[name] = base_camera @ np.asarray(pose)
        rospy.sleep(0.15)
    for name, pose in sorted(blocks.items()):
        print(name, "base xyz (m):", np.round(pose[:3, 3], 3))
    return blocks


def run_debug(stage, team, arm, detector, rospy, time_in_seconds):
    fk, ik = FK(), IK()
    print("Moving to observation pose...", flush=True)
    view = VIEW.copy()
    if team == "blue":
        view[[0, 2, 4]] *= -1
        view[6] = np.pi / 2 - view[6]
    q = move(arm, view)
    blocks = observe(arm, detector, fk, rospy, time_in_seconds)
    print("Detected", len(blocks), "different static blocks.", flush=True)
    if stage == "observe":
        return {"static_seen": len(blocks)}

    name, block = select_upper_cube(blocks, team)
    xyz = block[:3, 3]
    print("Selected", name, "base xyz (m):", np.round(xyz, 3), flush=True)
    _, ee = fk.forward(q)
    pre_xyz = xyz + [0.0, 0.0, 0.120]
    pre = solve(ik, q, ee[:3, :3], pre_xyz)
    arm.open_gripper()
    q = move(arm, pre)
    reached = fk.forward(q)[1][:3, 3]
    error_mm = float(np.linalg.norm(reached - pre_xyz) * 1000)
    print("Pre-grasp reached (m):", np.round(reached, 3),
          "error (mm):", round(error_mm, 2), flush=True)
    if stage == "pregrasp":
        return {"name": name, "pregrasp_error_mm": error_mm}
    if error_mm > 5:
        raise RuntimeError("Pre-grasp position error too large to descend.")

    # Refresh the same target from the stationary pre-grasp viewpoint.
    updated = observe(arm, detector, fk, rospy, time_in_seconds).get(name)
    if (updated is None or not np.isfinite(updated).all()
            or np.linalg.norm(updated[:3, 3] - xyz) > 0.015):
        raise RuntimeError("Selected cube is no longer visible at the expected position.")
    xyz = updated[:3, 3]
    rotation = grasp_rotation(updated)
    goals = []
    for dz in (0.120, 0.050, 0.003, 0.063):
        q = solve(ik, q, rotation, xyz + [0.0, 0.0, dz])
        goals.append(q)
    for q in goals[:3]:
        move(arm, q)

    arm.close_gripper()
    rospy.sleep(0.5)
    width = width_mm(arm)
    print("Width after close (mm):", round(width, 1), flush=True)
    if 38 <= width <= 65:
        move(arm, goals[3])
        rospy.sleep(0.5)
        print("Width after lift (mm):", round(width_mm(arm), 1))
        print("Check whether only the upper cube rose with the gripper.")
        return {"name": name, "grasp_width_mm": width, "postlift_width_mm": width_mm(arm)}
    else:
        arm.open_gripper()
        move(arm, goals[1])
        move(arm, goals[0])
        print("Unexpected width. Returned to pre-grasp pose.")
        return {"name": name, "grasp_width_mm": width, "held": False}


if __name__ == "__main__":
    from final import main
    main()
