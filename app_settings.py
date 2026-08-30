"""
app_settings.py — tiny JSON store for GUI settings that must survive a restart.

Plant parameters and controller gains are expensive to obtain (a step
identification run, or a careful manual tune) and are properties of the
physical rig, not of one session. Losing them every time the app closes makes
a bench session far slower than it needs to be.

The file lives under the user's home directory rather than next to the code:
when the app is frozen with PyInstaller, the code directory is a read-only
temporary extraction (`sys._MEIPASS`) that is deleted on exit.
"""
from __future__ import annotations

import json
import os
from pathlib import Path

APP_DIR_NAME = ".laserai"


def settings_dir() -> Path:
    """User-writable directory for this app's settings."""
    return Path(os.path.expanduser("~")) / APP_DIR_NAME


def settings_path(name: str) -> Path:
    return settings_dir() / name


def load(name: str) -> dict:
    """
    Read a settings file. Returns {} if it is missing, unreadable, corrupt, or
    does not contain a JSON object -- settings are a convenience, and a bad
    file must never stop the app from starting.
    """
    try:
        with open(settings_path(name), encoding="utf-8") as fh:
            data = json.load(fh)
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def save(name: str, data: dict) -> bool:
    """
    Write a settings file, creating the directory if needed.

    Writes to a temporary file and replaces the target, so an interrupted write
    cannot leave a half-written file behind. Returns True on success; callers
    treat failure as non-fatal.
    """
    try:
        settings_dir().mkdir(parents=True, exist_ok=True)
        target = settings_path(name)
        tmp = target.with_suffix(target.suffix + ".tmp")
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(data, fh, indent=2, sort_keys=True)
        os.replace(tmp, target)
        return True
    except Exception:
        return False
