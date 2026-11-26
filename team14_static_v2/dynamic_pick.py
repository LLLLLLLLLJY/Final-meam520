"""Observe the turntable from a stationary tool and intercept its near-side arc."""
import numpy as np
from static_pick import grasp_rotation, move, solve, width_mm


def observe_tracks(arm, detector, fk, rospy, clock, center, duration):
    camera = fk.forward(arm.get_positions())[1] @ detector.get_H_ee_camera()
    tracks = {}
    stop = clock() + duration
    while clock() < stop and not rospy.is_shutdown():
        t = clock()
        for name, relative in detector.get_detections():
            if "dynamic" not in name.lower():
                continue
            pose = camera @ np.asarray(relative)
            delta = pose[:2, 3] - center
            if not np.isfinite(pose).all() or not .08 < np.linalg.norm(delta) < .31:
                continue
            if not .210 < pose[2, 3] < .244:
                continue
            track = tracks.setdefault(name, [])
            if not track or np.linalg.norm(track[-1][1][:2, 3]-pose[:2, 3]) > .0005:
                track.append((t, pose.copy()))
        rospy.sleep(.08)
    return tracks


def estimate(track, center):
    if len(track) < 4 or track[-1][0]-track[0][0] < .45:
        return None
    t = np.array([v[0] for v in track])
    xy = np.array([v[1][:2, 3] for v in track])-center
    theta = np.unwrap(np.arctan2(xy[:, 1], xy[:, 0]))
    omega, phase = np.polyfit(t-t[-1], theta, 1)
    r = np.median(np.linalg.norm(xy,axis=1))
    noise = max(np.max(abs(theta - phase - omega*(t-t[-1])))*r,
                np.ptp(np.linalg.norm(xy,axis=1)))
    if not .015 < abs(omega) < 1.55 or noise > .008:
        return None
    return float(omega), float(phase), float(r), t[-1], track[-1][1]


def eta_at(angle, pred, now):
    omega, phase, _, ref, _ = pred
    sign = np.sign(omega)
    angle_now = phase + omega*(now-ref)
    return (sign*(angle-angle_now) % (2*np.pi))/abs(omega)


def pick_dynamic(arm, detector, fk, ik, rospy, clock, team, deadline):
    width = width_mm(arm)
    if not np.isfinite(width) or 38 <= width <= 65:
        raise RuntimeError("Inspect the gripper before starting a turntable pick.")
    side = 1 if team == "red" else -1
    center = np.array([0, side*.990])
    angle = -side*np.pi/2
    rotation = np.diag([1.,-1.,-1.])
    seed = np.array([side*1.25,-.13,0,-1.7,0,1.58,.78])
    view = solve(ik, seed, rotation, [.02,side*.70,.420])
    move(arm, view)
    arm.open_gripper()
    tracks = observe_tracks(arm, detector, fk, rospy, clock, center, 3.0)
    now = clock()
    choices = []
    for name, track in tracks.items():
        prediction = estimate(track, center)
        if prediction is None or now-prediction[3] > 1.2:
            continue
        eta = eta_at(angle, prediction, now)
        if 0 < eta < 5.:
            eta += 2*np.pi/abs(prediction[0])
        if now + eta + 12. < deadline and eta < 37.:
            choices.append((eta,name,prediction))
    # Six square targets offer four equivalent grasps. The two long moving
    # pieces remain available to the opponent and are excluded from this strategy.
    choices = [choice for choice in choices if "rect" not in choice[1].lower()]
    if not choices:
        return None
    eta, name, pred = min(choices, key=lambda row: row[0])
    omega, phase, radius, ref, latest = pred
    z = float(np.clip(latest[2, 3], .220,.232))
    target = np.array([0.,side*(.990-radius), z+.003])
    base_rotation = grasp_rotation(latest)
    c,s = np.cos(angle-phase),np.sin(angle-phase)
    predicted_rotation = np.array([[c,-s,0],[s,c,0],[0,0,1]]) @ base_rotation
    # Every quarter-turn is the same grasp for a cube. Select a reachable
    # wrist orientation when moving from the stationary observation pose.
    waypoint = np.eye(4)
    waypoint[:3, :3] = predicted_rotation
    predicted_rotation = grasp_rotation(waypoint)
    for yaw in (0.,np.pi/2,-np.pi/2,np.pi):
        c, s = np.cos(yaw), np.sin(yaw)
        rotation = np.array([[c,-s,0],[s,c,0],[0,0,1]]) @ predicted_rotation
        try:
            high = solve(ik, view, rotation, target + [0,0,.095])
            low = solve(ik, high, rotation, target)
            predicted_rotation = rotation
            break
        except RuntimeError:
            continue
    else:
        return None
    move(arm,high)
    gate_time = now + eta
    last_confirmed = ref
    # Watch again at the gate and use the freshest eligible observation.
    # Moving to the gate early keeps our finger-closing delay independent of the full approach.
    while clock() < gate_time - 3.5:
        update = observe_tracks(arm,detector,fk,rospy,clock,center,.8)
        revised = estimate(update.get(name,[]),center)
        if revised and abs(revised[0]-omega) < .07 and abs(revised[2]-radius)<.009:
            delta = eta_at(angle,revised,clock())
            if abs(clock()+delta-gate_time) < .50:
                gate_time = clock()+delta
                last_confirmed = revised[3]
        if rospy.is_shutdown() or clock()+8 >= deadline:
            return None
    if clock() - last_confirmed > 1.6 or gate_time - clock() < 1.0:
        return None
    move(arm,low)
    if clock() > gate_time+.16:
        move(arm,high)
        return None
    while clock()<gate_time-.12 and not rospy.is_shutdown():
        rospy.sleep(.025)
    if rospy.is_shutdown():
        return None
    arm.close_gripper()
    rospy.sleep(.2)
    width = width_mm(arm)
    if not np.isfinite(width):
        raise RuntimeError("Unknown finger width after turntable grasp.")
    if not 38 <= width <= 65:
        if width > 12:
            raise RuntimeError("Ambiguous turntable grasp width; inspect before opening.")
        arm.open_gripper()
        move(arm,high)
        return None
    move(arm,high)
    if not 38 <= width_mm(arm) <= 65:
        raise RuntimeError("Possible slip on lifting from turntable.")
    return dict(name=name,kind="cube",height=.05,rotation=predicted_rotation,grasp_z_offset=.003)
