"""Team 14 pick-and-place project. Start from the course final.launch environment."""
import argparse

from kinematics.calculateFK import FK
from kinematics.IK_position_null import IK
from static_pick import run_debug
from trial_log import TrackedArm, TrackedDetector, Trial


STAGES = ("match", "match-held", "stack", "stack-held", "cycle", "place-held",
          "dynamic", "observe", "pregrasp", "pick", "observe-goal")


def run(stage, team, arm, detector, rospy, clock, trial, deadline):
    from static_stack import run_static, place_dynamic
    from dynamic_pick import pick_dynamic

    if stage in ("observe", "pregrasp", "pick"):
        return run_debug(stage, team, arm, detector, rospy, clock)

    if stage not in ("match", "match-held", "dynamic"):
        return run_static(stage, team, arm, detector, rospy, clock, deadline=deadline)

    results = {"dynamic_placements": [], "dynamic_picks": 0}
    if stage != "dynamic":
        # Keep a terminal reserve for the approach, intercept, and return trip.
        # The hard motion deadline remains the full match duration.
        duration = deadline - trial.start
        static_end = deadline - min(60.0, duration / 3.0)
        task = "stack-held" if stage == "match-held" else "stack"
        result = run_static(task, team, arm, detector, rospy, clock, deadline=static_end)
        results["static"] = result
        trial.event("static_result", result=result)
        if not result.get("empty", False):
            results["reason"] = "Static phase ended with uncertain gripper state."
            return results
        if "unverified" in str(result.get("reason", "")):
            results["reason"] = "Inspect the tower before placing another block."
            return results

    fk, ik = FK(), IK()
    # Each call has a bounded observation and intercept window. A new attempt
    # needs enough time for a placement and cannot immediately retry forever.
    attempts = 0
    while not rospy.is_shutdown() and deadline - clock() >= 40 and attempts < 8:
        attempts += 1
        trial.event("dynamic_attempt", attempt=attempts)
        held = pick_dynamic(arm, detector, fk, ik, rospy, clock, team, deadline)
        if held is None:
            trial.event("dynamic_miss", attempt=attempts)
            if stage == "dynamic":
                break
            continue
        results["dynamic_picks"] += 1
        trial.event("dynamic_grasp", held=held)
        placement = place_dynamic(arm, detector, fk, ik, rospy, clock, team, held, deadline)
        results["dynamic_placements"].append(placement)
        trial.event("dynamic_place", result=placement)
        if not placement.get("empty", False) or not placement.get("placed", 0):
            results["reason"] = "Dynamic placement needs inspection before another attempt."
            break
        if stage == "dynamic":
            break
    results["dynamic_attempts"] = attempts
    return results


def main(competition=False):
    import rospy
    from core.interfaces import ArmController, ObjectDetector
    from core.utils import time_in_seconds

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", nargs="?", choices=STAGES, default="match")
    parser.add_argument("--hardware", action="store_true", help="use the course hardware in a lab session")
    parser.add_argument("--seconds", type=int, choices=range(30, 301), metavar="30..300", default=180,
                        help="planning time budget in ROS seconds; default qualification is 180")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("team14_final", anonymous=True)
    team = rospy.get_param("team", "")
    if team not in ("red", "blue"):
        raise SystemExit("Start the red or blue final-project environment first.")
    sim = bool(rospy.get_param("/use_sim_time", False))
    if not competition and sim == args.hardware:
        raise SystemExit("Mode mismatch: use default in Gazebo, --hardware in the lab.")
    arm_real = ArmController()
    detector_real = ObjectDetector()
    mode = "sim" if sim else "hardware"
    print("Team:", team, "| Environment:", mode, "| Stage:", args.stage, flush=True)
    msg = ("Confirm cube4_static is held above its source and the goal platform is empty."
           if args.stage.endswith("-held") else "Confirm the starting state and empty fingers.")
    input(msg + " Press ENTER to START: ")
    # The clock, perception, scoring decisions and all moves start after ENTER.
    trial = Trial(dict(team=team, stage=args.stage, environment=mode, budget_s=args.seconds),
                  time_in_seconds)
    deadline = trial.start + args.seconds
    arm = TrackedArm(arm_real, trial, deadline, rospy)
    detector = TrackedDetector(detector_real, trial)
    print("Run log:", trial.path, flush=True)
    try:
        details = run(args.stage, team, arm, detector, rospy, time_in_seconds, trial, deadline)
    except (Exception, KeyboardInterrupt) as exc:
        trial.finish("stopped", {"error": type(exc).__name__ + ": " + str(exc)})
        raise
    trial.finish("finished", details)
    print("Run result:", details)


if __name__ == "__main__":
    main()
