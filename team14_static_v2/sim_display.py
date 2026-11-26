"""Temporary display tuning for a running Gazebo Classic match, inside Ubuntu."""
import os
import re
import signal
import subprocess
import sys
import time
from contextlib import ExitStack
from pathlib import Path


def run(*args, timeout=20):
    return subprocess.run(args, check=True, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, text=True, timeout=timeout).stdout


def measured_rate():
    rows = []
    for line in run("gz", "stats", "-p", "-d", "10").splitlines():
        fields = [field.strip() for field in line.split(",")]
        if len(fields) == 4 and fields[-1] in ("T", "F"):
            if fields[-1] == "T":
                raise RuntimeError("Gazebo is paused. The test needs a running world.")
            rows.append((float(fields[1]), float(fields[2])))
    if len(rows) < 2 or rows[-1][1] - rows[0][1] < 3:
        raise RuntimeError("Too few Gazebo clock samples for comparison.")
    return (rows[-1][0] - rows[0][0]) / (rows[-1][1] - rows[0][1])


def process_info(p):
    try:
        if p.stat().st_uid != os.getuid():
            return None
        fields = (p / "stat").read_text().rsplit(") ", 1)[1].split()
        return (p / "comm").read_text().strip(), fields[19], fields[0]
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return None


def same_process(p, initial):
    info = process_info(p)
    return info is not None and info[:2] == initial[:2]


def resume(p, initial):
    if same_process(p, initial):
        try:
            os.kill(int(p.name), signal.SIGCONT)
        except ProcessLookupError:
            pass


def small_desktop(undo):
    # Only change an active single-output X11 desktop; leave multi-monitor arrangements alone.
    if os.environ.get("XDG_SESSION_TYPE", "") == "wayland":
        return False
    try:
        query = run("xrandr", "--query")
    except (OSError, subprocess.SubprocessError):
        return False
    active = re.findall(r"^(\S+) connected (?:primary )?(\d+x\d+)\+\d+\+\d+", query, re.M)
    if len(active) != 1:
        return False
    output, original = active[0]
    width, height = map(int, original.split("x"))
    if width <= 1280 or height <= 720:
        return False
    block = re.search(r"^" + re.escape(output) + r" connected[^\n]*\n((?:[ \t]+[^\n]*\n?)*)", query, re.M)
    modes = re.findall(r"^\s+(\d+x\d+)\s", block.group(1), re.M) if block else []
    target = next((mode for mode in ("1280x800", "1280x720", "1024x768") if mode in modes), None)
    if not target:
        return False
    run("xrandr", "--output", output, "--mode", target)
    def restore_size():
        if re.search(r"^" + re.escape(output) + r" connected (?:primary )?" +
                     re.escape(target) + r"\+", run("xrandr", "--query"), re.M):
            run("xrandr", "--output", output, "--mode", original)
    undo.callback(restore_size)
    print("Desktop:", original, "->", target, flush=True)
    return True


def main():
    print("Measuring the current Gazebo rate (10 wall seconds)...", flush=True)
    before = measured_rate()
    print("Before: {:.3f}x".format(before), flush=True)
    matches = []
    with ExitStack() as undo:
        changed = False
        for p in Path("/proc").glob("[0-9]*"):
            info = process_info(p)
            if not info:
                continue
            if info[0] == "rviz" and info[2] not in ("T", "t", "Z"):
                try:
                    os.kill(int(p.name), signal.SIGSTOP)
                except ProcessLookupError:
                    continue
                undo.callback(resume, p, info)
                print("Suspended RViz viewer", p.name, flush=True)
                changed = True
            elif info[0].startswith("python"):
                try:
                    cmd = (p / "cmdline").read_bytes().split(b"\x00")
                    where = str((p / "cwd").resolve(strict=True))
                except (OSError, RuntimeError):
                    continue
                if any(Path(os.fsdecode(arg)).name == "final.py" and
                       "/labs/final/" in str(Path(where) / os.fsdecode(arg)) for arg in cmd[1:]):
                    matches.append((p, info))
        changed = small_desktop(undo) or changed
        if not changed:
            print("No active RViz or smaller single-screen mode to test.")
            return
        time.sleep(2)
        after = measured_rate()
        print("After: {:.3f}x (was {:.3f}x)".format(after, before), flush=True)
        if before <= 0 or after < before * 1.05:
            print("No clear gain in this sample; restoring display settings.")
            return
        print("Keeping lighter graphics. Ctrl+C here restores the display.", flush=True)
        if matches:
            while any(same_process(p, info) for p, info in matches):
                time.sleep(1)
        else:
            input("Press Enter in this terminal to restore: ")
    print("Display restored.")


if __name__ == "__main__":
    def stop(signum, frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGHUP, stop)
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print("Display test ended.")
    except (OSError, subprocess.SubprocessError, RuntimeError) as exc:
        sys.exit(str(exc))
