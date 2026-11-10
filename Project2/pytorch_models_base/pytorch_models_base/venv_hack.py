
import sys
import os

venv=os.getenv("VIRTUAL_ENV")
print("Virtual env (python3.%d): "%sys.version_info.minor + str(venv))


if venv:  # getenv returns None (not "") when no venv is active
    sys.path.insert(0,venv+"/lib/python3.%d/site-packages"%sys.version_info.minor)
else:
    # No venv active (e.g. launched from a terminal where it wasn't sourced):
    # fall back to the course PyTorch venv: GPU one on machines with an NVIDIA GPU, else the CPU one.
    candidates = ["/cs-share/pradalier/venv/torch-cpu-ros2"]
    if os.path.exists("/dev/nvidia0"):
        candidates.insert(0, "/cs-share/pradalier/venv/torch-gpu-ros2")
    for fallback in candidates:
        site = fallback + "/lib/python3.%d/site-packages" % sys.version_info.minor
        if os.path.isdir(site):
            print("No virtual env active, using " + fallback)
            sys.path.insert(0, site)
            break



