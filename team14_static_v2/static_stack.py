"""Closed-loop static stacking in the robot base frame."""
import numpy as np
from kinematics.calculateFK import FK
from kinematics.IK_position_null import IK
from static_pick import VIEW, grasp_rotation, move, solve, width_mm


def scan(arm, detector, fk, rospy, clock):
    base_camera = fk.forward(arm.get_positions())[1] @ detector.get_H_ee_camera()
    seen = {}
    until = clock() + 1.1
    while clock() < until and not rospy.is_shutdown():
        for name, p in detector.get_detections():
            p = base_camera @ np.asarray(p)
            if p.shape == (4,4) and np.isfinite(p).all():
                seen.setdefault(name,[]).append(p)
        rospy.sleep(.1)
    result = {}
    for name, samples in seen.items():
        if len(samples) >= 2:
            pose = samples[-1].copy()
            pose[:3,3] = np.median([p[:3,3] for p in samples],axis=0)
            result[name] = pose
    return result


def go(arm, fk, ik, xyz, rotation, seed=None):
    q = arm.get_positions() if seed is None else seed
    q = solve(ik, q, rotation, np.asarray(xyz))
    move(arm,q)
    if np.linalg.norm(fk.forward(arm.get_positions())[1][:3,3] - xyz) > .005:
        raise RuntimeError("Cartesian check failed")


def capture(arm,detector,fk,ik,rospy,clock,team,goal):
    side = 1 if team == "red" else -1
    view=VIEW.copy()
    if side == -1:
        view[[0,2,4]] *= -1
        view[6] = np.pi/2-view[6]
    if goal:
        go(arm,fk,ik,[.562,side*.259,.56],np.diag([1.,-1.,-1.]),view)
    else:
        move(arm, view)
    return scan(arm,detector,fk,rospy,clock)


def tower(blocks, goal):
    candidates = [(name,p) for name,p in blocks.items()
                  if np.max(abs(p[:2,3]-goal)) < .10 and .21 < p[2,3] < .72]
    if not candidates:
        return np.array([*goal,.200])
    _, top = max(candidates,key=lambda item:item[1][2,3])
    return top[:3,3] + [0.,0.,.025]


def place_dynamic(arm,detector,fk,ik,rospy,clock,team,held,deadline=None):
    if deadline is not None and deadline - clock() < 12:
        raise RuntimeError("Too little time for a placement; keeping the gripper closed.")
    side = 1 if team == "red" else -1
    goal = np.array([.562,side*.259])
    rotation=fk.forward(arm.get_positions())[1][:3,:3].copy()
    current=fk.forward(arm.get_positions())[1][:3,3]
    if abs(current[1]) > .59:
        go(arm,fk,ik,[max(.14,current[0]),side*.55,max(.34,current[2])],rotation)
        current=fk.forward(arm.get_positions())[1][:3,3]
    go(arm,fk,ik,[current[0],current[1],max(.47,current[2]+.07)],rotation)
    before=capture(arm,detector,fk,ik,rospy,clock,team,True)
    if not any(np.max(abs(p[:2, 3] - goal)) < .125 for p in before.values()):
        # A held block can hide the center of an otherwise visible platform.
        # Recheck from a second position before the first placement.
        go(arm,fk,ik,[.475,goal[1],.52],np.diag([1.,-1.,-1.]))
        before.update(scan(arm,detector,fk,rospy,clock))
    rotation=fk.forward(arm.get_positions())[1][:3,:3].copy()
    blocks = {k:p for k,p in before.items() if k!=held["name"]}
    # Keep a clear retreat above each reachable tower. If the first column
    # grows beyond that approach height, use the adjacent free support.
    known_tops = getattr(arm, "known_goal_tops", {})
    for dx in (-.050, .050):
        column = goal + [dx, 0.]
        local = {k:p for k,p in blocks.items() if np.linalg.norm(p[:2,3]-column)<.043}
        previous = known_tops.get(dx)
        if previous is not None:
            name, xyz = previous
            if name not in local or np.linalg.norm(local[name][:3,3] - xyz) > .020:
                raise RuntimeError("Previous tower top is missing or shifted; keeping the gripper closed.")
        support=tower(local,column)
        if support[2] < .448:
            break
    else:
        raise RuntimeError("Both goal columns are above the planned approach height.")
    dz = held.get("height",.05)/2+held.get("grasp_z_offset",.003)
    dest=support + [0.,0.,dz+.003]
    # Center within the supporting face while arresting lateral drift from
    # a moving block which may have reached the fingers slightly off-center.
    dest[:2] += np.clip(column - support[:2], -.010, .010)
    # Keep transfer above the top face, then approach vertically in short stages.
    approach_height = .07 if support[2] >= .34 else .105
    for clearance in (approach_height,.035,.0):
        go(arm,fk,ik,dest+[0.,0.,clearance],rotation)
    if deadline is not None and deadline - clock() < 4:
        raise RuntimeError("Placement window expired before release; keeping the gripper closed.")
    arm.open_gripper()
    rospy.sleep(.30)
    if not np.isfinite(width_mm(arm)) or width_mm(arm) < 66:
        raise RuntimeError("Open command not confirmed.")
    go(arm,fk,ik,dest+[0.,0.,approach_height],rotation)
    after=capture(arm,detector,fk,ik,rospy,clock,team,True)
    p=after.get(held["name"])
    expected=support+[0,0,held.get("height",.05)/2]
    verified = p is not None and np.linalg.norm(p[:2,3]-expected[:2])<.026 and abs(p[2,3]-expected[2])<.018
    if verified:
        known_tops[dx] = (held["name"], p[:3,3].copy())
        arm.known_goal_tops = known_tops
    return dict(placed=int(verified),empty=True,tower=tower(after,goal).tolist(),
                reason="verified" if verified else "released but unverified")


def run_static(stage,team,arm,detector,rospy,clock,confirm=input,deadline=None):
    fk,ik=FK(),IK()
    goal=np.array([.562,.259 if team=="red" else -.259])
    source=np.array([.562,-.169 if team=="red" else .169])
    placed=0
    top=[*goal,.20]
    done=set()
    if stage=="observe-goal":
        blocks=capture(arm,detector,fk,ik,rospy,clock,team,True)
        return dict(placed=0,tower=tower(blocks,goal).tolist(),empty=True,reason="observation")
    if stage.endswith("-held"):
        width=width_mm(arm)
        ee = fk.forward(arm.get_positions())[1]
        over_source = np.max(np.abs(ee[:2,3] - source)) < .14
        if not (38 <= width <= 65 and over_source and .307 < ee[2,3] < .38 and ee[2,2] < -.94):
            return dict(placed=0,empty=False,tower=top,reason="expected lifted upper cube at source not confirmed")
        held=dict(name="cube4_static",height=.05,grasp_z_offset=.003)
        result=place_dynamic(arm,detector,fk,ik,rospy,clock,team,held,getattr(arm,"deadline",deadline))
        if not result["placed"] or stage=="place-held":
            return result
        placed+=1; done.add(held["name"]); top=result["tower"]
    else:
        width = width_mm(arm)
        if not np.isfinite(width) or 38<=width<=65:
            return dict(placed=0,tower=top,empty=False,reason="start requires empty fingers")
        arm.open_gripper()
    while len(done)<4 and not rospy.is_shutdown():
        if deadline is not None and deadline-clock()<18:
            break
        blocks=capture(arm,detector,fk,ik,rospy,clock,team,False)
        choices=[(n,p) for n,p in blocks.items() if n not in done and "static" in n.lower()
                 and np.max(np.abs(p[:2,3]-source)) < .14 and .204 < p[2,3]<.31]
        if not choices:
            break
        # The cube on the rectangle must clear its support first.
        n,p=max(choices,key=lambda item:(round(item[1][2,3]/.018), "rect" in item[0].lower()))
        xyz=p[:3,3].copy()
        rotation=grasp_rotation(p)
        if "rect" in n.lower():
            axis=p[:2,0]
            if np.linalg.norm(axis)>.9:
                yaw=np.arctan2(axis[1],axis[0])
                yaw=(yaw+np.pi/2)%np.pi-np.pi/2
                c,s=np.cos(yaw),np.sin(yaw)
                rotation=np.array([[c,s,0],[s,-c,0],[0,0,-1.]])
        go(arm,fk,ik,xyz+[0,0,.12],rotation)
        observed=scan(arm,detector,fk,rospy,clock).get(n)
        if observed is None or np.linalg.norm(observed[:3,3]-xyz)>.015:
            break
        xyz=observed[:3,3]
        for dz in (.05,.003):
            go(arm,fk,ik,xyz+[0,0,dz],rotation)
        arm.close_gripper(); rospy.sleep(.3)
        width=width_mm(arm)
        if not np.isfinite(width):
            return dict(placed=placed,empty=False,tower=top,reason="unknown finger width")
        if not 38<=width<=65:
            arm.open_gripper()
            go(arm,fk,ik,xyz+[0,0,.12],rotation)
            break
        go(arm,fk,ik,xyz+[0,0,.088],rotation)
        if not 38<=width_mm(arm)<=65:
            return dict(placed=placed,empty=False,tower=top,reason="possible slip")
        result=place_dynamic(arm,detector,fk,ik,rospy,clock,team,dict(name=n,height=.05,grasp_z_offset=.003),getattr(arm,"deadline",deadline))
        placed+=result["placed"];top=result["tower"]
        if not result["placed"] or stage=="cycle":
            return dict(result,placed=placed)
        done.add(n)
    return dict(placed=placed,tower=top,empty=True,reason="static phase complete")
