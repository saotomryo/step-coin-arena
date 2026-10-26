"""Explicit portable resource storage, with legacy external-volume protection."""
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def resource_root(project=None):
    project = Path(project) if project is not None else ROOT
    override = os.environ.get("XROBOCON_RESOURCE_ROOT")
    if override:
        path = Path(override).expanduser()
        path = path if path.is_absolute() else project / path
    else:
        config = project / "ros2/storage.local.json"
        if not config.is_file():
            raise RuntimeError("Set XROBOCON_RESOURCE_ROOT or run scripts/setup_ros2.py --resource-root <directory>")
        spec = json.loads(config.read_text())
        if spec.get("volume") and not os.path.ismount(spec["volume"]):
            raise RuntimeError("Configured external volume is not mounted; no internal fallback")
        path = Path(spec["resource_root"]).expanduser()
        path = path if path.is_absolute() else project / path
    # Do not create a mount-point directory when a macOS external disk is absent.
    if path.parts[1:2] == ("Volumes",) and len(path.parts) > 2:
        if not os.path.ismount(Path(path.anchor) / path.parts[1] / path.parts[2]):
            raise RuntimeError("Configured external volume is not mounted; no internal fallback")
    return path.resolve()
