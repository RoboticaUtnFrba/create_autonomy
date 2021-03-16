#!/usr/bin/env python3

"""
Author:     Diego Maldonado (@dmnunez1993)
Maintainer: Emiliano Borghi (@eborghi10)
"""

import os
import subprocess
from subprocess import PIPE

KNOWN_NVIDIA_ERRORS = [
    ("Failed to initialize NVML: Driver/library version mismatch"),
    ("NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver. "
     "Make sure that the latest NVIDIA driver is installed and running."),
]

def is_nvidia():
    try:
        everything, _ = subprocess.Popen(["nvidia-smi", "--query-gpu=driver_version", "--format=noheader,csv"],
                                         stdout=PIPE, stderr=PIPE).communicate()

        return (False if everything.strip() in KNOWN_NVIDIA_ERRORS else True)
    except OSError:
        return False

    return False

def run_command(command):
    subprocess.call(command, shell=True)

def get_repo_root():
    return subprocess.check_output('git rev-parse --show-toplevel'.split()).strip().decode()

def get_uid():
    return os.getuid()

def get_user():
    return os.getlogin()

def create_directory(directory):
    # Check existance before creating the directory
    if (os.path.isdir(directory)): return
    run_command(f"mkdir -p {directory}")
    run_command(f"sudo chown {get_user()}:{get_user()} {directory}")

def mount_resource(device):
    # Mount hardware resource if it exists in the host
    if os.path.exists(device):
        return f"--volume=\"{device}:{device}:shared\""
    return ""
