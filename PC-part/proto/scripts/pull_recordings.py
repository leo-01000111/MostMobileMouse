"""Pull new recordings from the phone into recordings/ and validate them.

usage: python PC-part/proto/scripts/pull_recordings.py
Only rec_* folders not already present locally are pulled. Needs adb (USB debugging).
"""
import os
import shutil
import subprocess
import sys
from pathlib import Path

REMOTE = "/sdcard/Android/data/eu.leongorecki.deskmouse.recorder/files/recordings"
ROOT = Path(__file__).resolve().parents[3]
LOCAL = ROOT / "recordings"


def adb() -> str:
    exe = shutil.which("adb")
    if exe:
        return exe
    return str(Path(os.environ.get("LOCALAPPDATA", "")) / "Android/Sdk/platform-tools/adb.exe")


def main():
    env = dict(os.environ, MSYS_NO_PATHCONV="1")
    out = subprocess.run([adb(), "shell", "ls", REMOTE], capture_output=True, text=True, env=env, check=True).stdout
    remote = sorted(n.strip() for n in out.split() if n.strip().startswith("rec_"))
    new = [n for n in remote if not (LOCAL / n / "meta.json").exists()]
    print(f"{len(remote)} on phone, {len(new)} new")
    for n in new:
        subprocess.run([adb(), "pull", f"{REMOTE}/{n}", str(LOCAL)], env=env, check=True)
    if new:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from validate_recording import check
        ok = all([check(LOCAL / n) for n in new])
        sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
