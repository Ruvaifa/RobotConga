"""Pytest configuration and pythonpath resolution for Robot Conga."""

from pathlib import Path
import sys

# Add project root directory to sys.path so robot_conga is importable without pip install -e .
project_root = Path(__file__).resolve().parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

