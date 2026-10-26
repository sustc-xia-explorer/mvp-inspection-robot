#!/usr/bin/env python3
"""Apply the reviewed FW-mini timeout patch after checking both source hashes."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time


def main():
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--driver-root', type=Path, default=repo.parent/'FW-mini-ros')
    args = parser.parse_args()
    root = args.driver_root.expanduser().resolve()
    original = json.loads((repo/'hardware/driver_original_sha256.json').read_text())
    patched = json.loads((repo/'hardware/driver_patched_sha256.json').read_text())
    current = {f: hashlib.sha256((root/f).read_bytes()).hexdigest() for f in original}
    if current == patched:
        print('Watchdog patch already applied:', root)
        return
    if current != original:
        raise SystemExit('Driver differs from the reviewed version. No files changed. Compare hardware/fw_mini_watchdog.patch manually.')
    command = ['patch', '--batch', '--forward', '-p1', '-i', str(repo/'hardware/fw_mini_watchdog.patch')]
    subprocess.run(command+['--dry-run'], cwd=root, check=True)
    backup = root/('.neupan_watchdog_backup_'+time.strftime('%Y%m%d_%H%M%S'))
    for f in original:
        target = backup/f
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(root/f, target)
    subprocess.run(command, cwd=root, check=True)
    assert {f: hashlib.sha256((root/f).read_bytes()).hexdigest() for f in original} == patched
    print('Applied. Rebuild the driver workspace. Backup:', backup)


if __name__ == '__main__': main()
