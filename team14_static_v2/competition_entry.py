"""Optional course labs/final/final.py launcher. Keep team14_static_v2/ beside it."""
import runpy
import sys
from pathlib import Path


if __name__ == "__main__":
    folder = Path(__file__).resolve().parent / "team14_static_v2"
    sys.path.insert(0, str(folder))
    entry = runpy.run_path(str(folder / "final.py"))
    entry["main"](competition=True)
