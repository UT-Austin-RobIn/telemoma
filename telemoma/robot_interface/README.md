# TeleMoMa in Sim and Real

## Using TeleMoMa in simulation environments
Here we provide the instructions on how to setup TeleMoMa with OmniGibson. 

### iGibson 
[iGibson](https://stanfordvl.github.io/iGibson/) is a simulation environment providing fast visual rendering and physics simulation based on Bullet. It is packed with a dataset with hundreds of large 3D environments reconstructed from real homes and offices, and interactive objects that can be pushed and actuated.

#### Setup Instructions
We provide a script to test TeleMoMa in an empty iGibson environment so that users can quickly test their installation. This also means that we can easily install it using pip,
```
(telemoma) > pip install igibson
``` 
For a more complete usage of TeleMoMa in a simulation environment, please checkout instructions on [OmniGibson](#omnigibson) below. 

#### Usage Instructions
To get started, simply run, 
```
(telemoma) > python telemoma/demo.py --robot <tiago or fetch> --teleop_config <path to telemoma config>  
```
Where the ```--robot``` can be chosen between tiago and fetch, and the ```--teleop_config``` is the config file specifying the input modality for teleoperation. Sample configurations are provided in the [telemoma/configs](../configs/) folder. 

### OmniGibson

[OmniGibson](https://behavior.stanford.edu/omnigibson) is the official simulation platform for [BEHAVIOR-1K](https://behavior.stanford.edu/behavior-1k), and provides a novel environment that supports robotics tasks via realistic physics simulation and rendering of rigid bodies, deformable bodies, and liquids. 

#### Setup Instructions
Please follow OmniGibson's [official installation guide](https://behavior.stanford.edu/omnigibson/getting_started/installation.html) to install OmniGibson from source. Please make sure you install the nightly build (i.e. clone the repository from \<og-develop> branch).

#### Usage Instructions
OmniGibson provides demo scripts of using TeleMoMa for robot teleoperation. To get started, simply run, 
```
(omnigibson) > python omnigibson/examples/teleoperation/robot_teleoperate_demo.py 
```
and then follow the prompt to choose the robot and teleoepration method as you desire.

## Using TeleMoMa with real robots
TeleMoMa can easily be integrated with real robots. Code for a real <b>PAL Tiago++</b> and <b>Toyota HSR</b> robot is provided in the [telemoma-real](https://github.com/UT-Austin-RobIn/telemoma/tree/telemoma-real) branch.

We use ROS for communicating with the robots, thus requiring some [additional dependancies](https://github.com/UT-Austin-RobIn/telemoma/blob/telemoma-real/real-robots.md). 

## Setting Up TeleMoMa with your own robot
TeleMoMa can be quickly set-up on a new robot through a simple gym environment acting as an interface between TeleMoMa and the robot. The gym interface must use the following convention for actions and observations,
```
# actions
- right: end-effector cartesian position and orientation delta + grasp (7-dimensional) 
- left: end-effector cartesian position and orientation delta + grasp. Ignored for single arm robots (7-dimensional)
- base: X, Y linear velocity and Z angular velocity (3-dimensional)
- torso: torso delta (1-dimensional)

# observations
- right: end-effector position and quarternion wrt robot base + gripper open/close state (8-dimensional)
- left: end-effector position and quarternion wrt robot base + gripper open/close state. None for single arm robots (8-dimensional)
- base: X, Y position delta and Z angular delta wrt the initial position of the robot at reset. Only required for vision teleop mode, can be None for rest (3-dimensional)
- torso: torso qpos (1-dimensional)
```

An example sim environment is provided [here](igibson/igibson_env.py) which can be run with the ```telemoma/demo_igibson.py``` script as shown above. A real robot environment on a Tiago++ is available [here](https://github.com/UT-Austin-RobIn/telemoma/blob/telemoma-real/telemoma/robot_interface/tiago/tiago_gym.py). 


We are always looking to extend the support of TeleMoMa to more robots. If you use TeleMoMa with a robot not currently supported and would like to contribute, feel free to open a new PR. 