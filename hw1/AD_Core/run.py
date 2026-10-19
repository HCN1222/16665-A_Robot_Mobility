"""Run the newly implemented Core simulations from any working directory."""
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parent
for name in ['kinematics.py', 'lane_change.py']:
    subprocess.run([sys.executable, str(root/name)], cwd=root, check=True)
print('All implemented numerical checks completed; read lane_validation.json for scope.')
