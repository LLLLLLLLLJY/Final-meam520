"""Capture only values provided by the course interfaces and match routine."""
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

import numpy as np


def plain(v):
    if isinstance(v, dict):
        return {str(k): plain(x) for k, x in v.items()}
    if isinstance(v, np.ndarray):
        return plain(v.tolist())
    if isinstance(v, (list, tuple)):
        return [plain(x) for x in v]
    if isinstance(v, np.generic):
        return plain(v.item())
    if isinstance(v, float) and not math.isfinite(v):
        return None
    return v


class Trial:
    def __init__(self, meta, clock):
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.path = Path.home() / "meam520_project_runs" / (stamp + "_" + uuid4().hex[:8])
        self.path.mkdir(parents=True)
        self.stream = (self.path / "events.jsonl").open("x")
        self.clock = clock
        self.start = clock()
        self.wall = time.monotonic()
        self.meta = meta
        self.event("start", **meta)

    def event(self, kind, **data):
        row = dict(event=kind, elapsed_ros_s=self.clock()-self.start,
                   elapsed_wall_s=time.monotonic()-self.wall, **data)
        self.stream.write(json.dumps(plain(row), allow_nan=False) + "\n")
        self.stream.flush()

    def finish(self, status, details):
        result = dict(self.meta, status=status, details=details,
                      elapsed_ros_s=self.clock()-self.start,
                      elapsed_wall_s=time.monotonic()-self.wall)
        self.event("finish", status=status, details=details)
        (self.path / "result.json").write_text(json.dumps(plain(result), indent=2, allow_nan=False))
        self.stream.close()


class TrackedArm:
    def __init__(self, arm, trial, deadline, rospy):
        self.arm, self.trial, self.deadline, self.rospy = arm, trial, deadline, rospy

    def get_positions(self):
        return self.arm.get_positions()

    def get_gripper_state(self):
        return self.arm.get_gripper_state()

    def check_time(self, allowance):
        remaining = self.deadline - self.trial.clock()
        if self.rospy.is_shutdown() or remaining <= allowance:
            raise RuntimeError("Stopping before the next command: {:.2f} ROS seconds remaining.".format(remaining))

    def safe_move_to_position(self, q, *args, **kwargs):
        self.check_time(4.0)
        self.trial.event("move_begin", target_q=q)
        result = self.arm.safe_move_to_position(q, *args, **kwargs)
        self.trial.event("move_end", measured_q=self.get_positions())
        return result

    def open_gripper(self):
        self.check_time(1.0)
        self.trial.event("open_requested")
        result = self.arm.open_gripper()
        self.trial.event("opened", gripper=self.get_gripper_state())
        return result

    def close_gripper(self):
        self.check_time(1.0)
        self.trial.event("close_requested")
        result = self.arm.close_gripper()
        self.trial.event("closed", gripper=self.get_gripper_state())
        return result


class TrackedDetector:
    def __init__(self, detector, trial):
        self.detector, self.trial = detector, trial
        self.last = float("-inf")

    def get_H_ee_camera(self):
        return self.detector.get_H_ee_camera()

    def get_detections(self):
        blocks = list(self.detector.get_detections())
        now = self.trial.clock()
        if now - self.last >= 0.25:
            self.trial.event("camera", blocks={k: p for k, p in blocks})
            self.last = now
        return blocks
