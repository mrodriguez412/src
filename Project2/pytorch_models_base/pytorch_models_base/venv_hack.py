
import sys
import os

venv=os.getenv("VIRTUAL_ENV")
print("Virtual env (python3.%d): "%sys.version_info.minor + str(venv))


if venv != "":
    sys.path.insert(0,venv+"/lib/python3.%d/site-packages"%sys.version_info.minor)
else:
    import socket
    if socket.gethostname().startswith("gtlpc1"):
        # Ugly hack to bypass roslaunch removing the venv path
        sys.path.insert(0,"/cs-share/pradalier/venv/tf-cpu-ros2/lib/python3.%d/site-packages"%sys.version_info.minor)

    if socket.gethostname() in ["galadriel","balrog","orodruin"]:
        # Ugly hack to bypass roslaunch removing the venv path
        sys.path.insert(0,"/opt/venv/tf-gpu-ros2/lib/python3.10/site-packages")



