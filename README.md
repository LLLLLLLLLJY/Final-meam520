# MEAM 5200 Final Project · Team 14

The Panda stacks blocks on its team's goal platform. It starts with the static blocks, then tracks moving cubes on the turntable. A match runs for up to 180 seconds of ROS time.

## Run a fresh simulation

Use the course Ubuntu VM with the final-project environment already built. Clone the repository or choose **Code → Download ZIP** on GitHub and extract it. Open a terminal in the folder containing this README and run:

```bash
bash team14_static_v2/install.sh
source ~/meam520_ws/devel_isolated/setup.bash
roslaunch meam520_labs final.launch team:=red
```

For the blue side, use `team:=blue`. Keep the launch terminal open. In a second terminal, run:

```bash
source ~/meam520_ws/devel_isolated/setup.bash
cd ~/meam520_ws/src/meam520_labs/labs/final/team14_static_v2
python3 final.py match
```

Start with an empty gripper and press **Enter** at the start prompt. The terminal prints the run directory under `~/meam520_project_runs/`. Look in `events.jsonl` for motion and perception events and in `result.json` for the outcome.

## Course competition launcher

To install the launcher at the course's expected `labs/final/final.py` path, run `bash team14_static_v2/install.sh --competition` from the downloaded folder. The installer backs up an existing entry point before replacing it. See the [Chinese operating notes](team14_static_v2/README.txt) for hardware setup and individual test modes.
