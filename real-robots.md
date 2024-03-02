# Real Robot Requirements and Setup

## System Requirements
- Ubuntu 20.04
- Python 3.9+
- ROS Noetic

Other system configurations may work, but we haven’t tested them extensively and may only be able to provide limited support.

## Additional Installations
Note that in addition to the installation instructions provided in the [README](README.md), following packages are required for using TeleMoMa with supported real robots - Tiago++ and HSR.

### ROS Noetic
Follow the guide provided at the official [ROS wiki](https://wiki.ros.org/noetic/Installation/Ubuntu) to install ROS Noetic on your system.

### Tracikpy

We use [Tracikpy](https://github.com/mjd3/tracikpy) for computing the IK of the robots. The package is simple to set up and only requires the URDF of the robot to get started.
```
(telemoma) > git clone https://github.com/mjd3/tracikpy.git
(telemoma) > cd tracikpy
(telemoma) > pip install -e .
```

## Usage
We provide a demo script at ```telemoma/demo_real.py``` to get started with TeleMoMa on a real Tiago++ or HSR. The script can be run as follows,
```
(telemoma) > python telemoma/demo_real.py --robot <tiago or hsr> --teleop_config <path-to-config>
``` 