import runpy
from pathlib import Path

HERE = Path(__file__).resolve().parent

SCRIPTS = [
    "sec3_test_system.py",
    "sec4_metrics.py",
    "sec5_each_part.py",
    "sec6_all_together.py",
    "sec7_real_world.py",
    "sec8_tuning.py",
    "sec9_final_pid.py",
]

for name in SCRIPTS:
    print(f"\n{'=' * 20} {name} {'=' * 20}")
    runpy.run_path(str(HERE / name), run_name="__main__")
