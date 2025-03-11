# Real Robot Requirements and Setup

## System Requirements
- Ubuntu 20.04
- Python 3.9+
- ROS Noetic

Other system configurations may work, but we haven’t tested them extensively and may only be able to provide limited support.

## Franka Dependecies
We use [Deoxys](https://zhuyifengzju.github.io/deoxys_docs/html/index.html) for interfacing with the Franka. Follow their [instructions](https://zhuyifengzju.github.io/deoxys_docs/html/installation/system_prerequisite.html) to set it up and change the path to the deoxys folder [here](https://github.com/UT-Austin-RobIn/telemoma/blob/897738eaded50bd68ce94bee7efb367e66a2231a/telemoma/robot_interface/franka/franka_gym.py#L8) appropriately.

## Tiago and HSR Dependencies
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

## HSR Specific Dependencies
We use ```hsrb_interface``` to control the HSR. If you would like to run TeleMoMa on the real HSR, please follow the documentation from Toyota to install the dependencies.

Alternatively, you want to use an open source HSR simulator like [this one](https://github.com/hsr-project/tmc_wrs_docker), you can get ```hsrb_interface``` [here](https://github.com/hsr-project/hsrb_interfaces/tree/master/hsrb_interface_py).

## Usage
We provide a demo script at ```telemoma/demo_real.py``` to get started with TeleMoMa on a real Tiago++ or HSR. The script can be run as follows,
```
(telemoma) > python telemoma/demo_real.py --robot <tiago or hsr> --teleop_config <path-to-config>
``` 