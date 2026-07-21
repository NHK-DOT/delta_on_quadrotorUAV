#!/usr/bin/env python3
"""Deploy the minimal 78arm Jetson Python 3.6 sampler package over SSH/SFTP.

Run this from the Windows/Linux development host. It intentionally copies only
the runtime pieces needed by the Jetson sampler instead of mirroring generated
vision exports or saved run artifacts.
"""

import argparse
import os
import posixpath
import stat
import sys
from pathlib import Path

import paramiko


THIS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = THIS_DIR.parents[2]


COPY_ITEMS = [
    Path("Delta_Gcode_Servo/real_machine_test/jetson_py36"),
    Path("Delta_Gcode_Servo/real_machine_test/workspace_model_tools.py"),
    Path("Delta_Gcode_Servo/real_machine_test/apriltag_gamepad_workspace_sampler.py"),
    Path("Delta_Gcode_Servo/real_machine_test/APRILTAG_GAMEPAD_WORKSPACE_SAMPLER_README.md"),
    Path("Delta_Gcode_Servo/real_machine_test/run_apriltag_workspace_sampler_jetson.sh"),
    Path("Delta_Gcode_Servo/delta_gcode_servo"),
    Path("bt_8bitdo_min/src/evdev_gamepad.py"),
    Path("bt_8bitdo_min/src/servo_driver.py"),
    Path("bt_8bitdo_min/config"),
    Path("bt_8bitdo_min/deploy/install_ubuntu18.sh"),
    Path("bt_8bitdo_min/README.md"),
    Path("lx225_tool_demo/config/lx225_tool.demo.toml"),
    Path("Dual_Camera_HandEye/output/calibration_result.json"),
    Path("Dual_Camera_HandEye/src/dual_handeye"),
    Path("Dual_Camera_HandEye/tools"),
]


SKIP_DIR_NAMES = {
    "__pycache__",
    ".pytest_cache",
    ".mypy_cache",
    "samples",
    "apriltag_workspace_samples",
    "workspace_model_output",
}


REMOTE_REMOVE_ITEMS = [
    "bt_8bitdo_min/deploy/run_control_bt.sh",
    "bt_8bitdo_min/deploy/run_log_test.sh",
    "bt_8bitdo_min/deploy/run_mapping_check.sh",
    "bt_8bitdo_min/deploy/run_serial_check.sh",
    "bt_8bitdo_min/deploy/run_serial_move_check.sh",
    "bt_8bitdo_min/deploy/run_show_state.sh",
    "bt_8bitdo_min/docs",
    "bt_8bitdo_min/src/check_gamepad_mapping.py",
    "bt_8bitdo_min/src/check_serial_readonly.py",
    "bt_8bitdo_min/src/config.py",
    "bt_8bitdo_min/src/gamepad_controller.py",
    "bt_8bitdo_min/src/joystick_linux.py",
    "bt_8bitdo_min/src/kinematics.py",
    "bt_8bitdo_min/src/move_serial_smoke.py",
    "bt_8bitdo_min/src/run_real_machine_bt.py",
    "bt_8bitdo_min/src/servo_mapping.py",
    "bt_8bitdo_min/src/show_control_state.py",
    "bt_8bitdo_min/src/test_gamepad_once.py",
]


def should_skip(path: Path) -> bool:
    parts = set(path.parts)
    if parts.intersection(SKIP_DIR_NAMES):
        return True
    name = path.name
    if name.endswith((".pyc", ".pyo", ".log", ".tmp")):
        return True
    return False


def mkdir_p(sftp, remote_dir):
    if remote_dir in ("", "/"):
        return
    parts = []
    current = remote_dir
    while current not in ("", "/"):
        parts.append(current)
        current = posixpath.dirname(current)
    for directory in reversed(parts):
        try:
            sftp.stat(directory)
        except IOError:
            sftp.mkdir(directory)


def chmod_if_script(sftp, remote_path):
    if remote_path.endswith(".sh") or posixpath.basename(remote_path) in {
        "jetson_workspace_preflight.py",
        "jetson_apriltag_workspace_sampler_py36.py",
    }:
        try:
            attrs = sftp.stat(remote_path)
            sftp.chmod(remote_path, attrs.st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
        except IOError:
            pass


def put_file(sftp, local_path, remote_path):
    mkdir_p(sftp, posixpath.dirname(remote_path))
    sftp.put(str(local_path), remote_path)
    chmod_if_script(sftp, remote_path)


def put_tree(sftp, local_root, remote_root):
    count = 0
    for path in sorted(local_root.rglob("*")):
        rel = path.relative_to(local_root)
        if should_skip(rel):
            continue
        remote_path = posixpath.join(remote_root, rel.as_posix())
        if path.is_dir():
            mkdir_p(sftp, remote_path)
            continue
        if path.is_file():
            put_file(sftp, path, remote_path)
            count += 1
    return count


def exec_checked(client, command):
    stdin, stdout, stderr = client.exec_command(command)
    del stdin
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    rc = stdout.channel.recv_exit_status()
    return rc, out, err


def shell_quote(text):
    return "'" + str(text).replace("'", "'\"'\"'") + "'"


def deploy(args):
    password = args.password or os.environ.get("JETSON_PASSWORD")
    if not password:
        print("Missing password. Pass --password or set JETSON_PASSWORD.", file=sys.stderr)
        return 2

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        hostname=args.host,
        username=args.user,
        password=password,
        timeout=10,
        look_for_keys=False,
        allow_agent=False,
    )
    try:
        sftp = client.open_sftp()
        try:
            mkdir_p(sftp, args.remote_root)
            total = 0
            for rel in COPY_ITEMS:
                local_path = PROJECT_ROOT / rel
                if not local_path.exists():
                    print("skip missing: %s" % local_path)
                    continue
                remote_path = posixpath.join(args.remote_root, rel.as_posix())
                if local_path.is_dir():
                    print("copy dir:  %s -> %s" % (rel.as_posix(), remote_path))
                    total += put_tree(sftp, local_path, remote_path)
                else:
                    print("copy file: %s -> %s" % (rel.as_posix(), remote_path))
                    put_file(sftp, local_path, remote_path)
                    total += 1
            print("copied files: %d" % total)
        finally:
            sftp.close()

        remove_targets = " ".join(
            shell_quote(posixpath.join(args.remote_root, item))
            for item in REMOTE_REMOVE_ITEMS
        )
        if remove_targets:
            rc, out, err = exec_checked(client, "rm -rf -- %s" % remove_targets)
            if out:
                print(out.rstrip())
            if err:
                print(err.rstrip(), file=sys.stderr)
            if rc != 0:
                print("remote cleanup failed rc=%d" % rc, file=sys.stderr)
                return rc
            print("remote old 8BitDo files removed")

        cmd = (
            "cd {root}/Delta_Gcode_Servo/real_machine_test/jetson_py36 && "
            "chmod +x run_sampler_py36_jetson.sh *.py && "
            "python3 -m py_compile jetson_workspace_common.py "
            "jetson_workspace_preflight.py jetson_apriltag_workspace_sampler_py36.py "
            "jetson_wrench_image_follower_py36.py jetson_wrench_grasp_demo_py36.py "
            "jetson_gamepad_raw_jog_py36.py"
        ).format(root=args.remote_root)
        rc, out, err = exec_checked(client, cmd)
        if out:
            print(out.rstrip())
        if err:
            print(err.rstrip(), file=sys.stderr)
        if rc != 0:
            print("remote compile failed rc=%d" % rc, file=sys.stderr)
            return rc
        print("remote compile ok")
        return 0
    finally:
        client.close()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.1.174")
    parser.add_argument("--user", default="nvidia")
    parser.add_argument("--password", default="")
    parser.add_argument("--remote-root", default="/home/nvidia/Desktop/78arm")
    return parser.parse_args(argv)


def main(argv=None):
    return deploy(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
