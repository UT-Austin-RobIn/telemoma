# Human Interface
Here we provide the instructions on how to setup different hardware interfaces compatible with TeleMoMa. 

## Realsense Camera
### Setup Instructions
We use ```pyrealsense2``` to communicate with a connected realsense camera. In the conda environment run,
```
pip install pyrealsense2
```

We also provide a ROS listener pipeline for the camera. Launch a realsense2 ROS node in a separate window, and in [this](https://github.com/UT-Austin-RobIn/telemoma/blob/main/telemoma/configs/only_vision.py) config, comment out line 4 and uncomment lines 6 and 7 before running the demo script.

### Usage Instructions
For teleoperating the robot only using vision, use ```telemoma/configs/only_vision.py``` configuration.

When the script starts, stand in front of the camera for the first 10s so that the reference can be recorded. The base motion is controlled by the hip -- the angular rotation is measured globally whereas the translation is measured as deltas in human motion. The arms are commanded by the change in wrist poses with respect to the hip and the gripper open/close commands are sent if the thumb-index distance is above/below a fixed threshold. The distance between ankles and hip is used to command the torso.

Note: For most accurate hand detection, make sure hands are clearly visible by keeping the palms of the hands facing the camera.

## Oculus
### Setup Instructions
We use [oculus_reader](https://github.com/rail-berkeley/oculus_reader) repository to read information from the oculus headset. Please vision the repository and follow the setup instructions.

### Usage Instructions
For teleoperating the robot only using Oculus, use ```telemoma/configs/only_vr.py``` configuration.

Oculus hand controllers are tracked with respect to the headset. To start controlling the arms, move the corresponding controller while holding the trigger button. The gripper button can be used to toggle between opening and closing the grippers and the joysticks are used to control the translation (right controller) and rotation (left controller) of the base. 

Note 1: If the mapping of oculus to the robot hand seems off, hold the controllers such that the controller's circular hole is parallel to the floor and click the joysticks. This will reset the coordinate axes.

Note 2: If using Oculus for controlling arms and Vision for controlling the base, place the oculus on top of your head (without covering your eyes).

## Spacemouse
### Setup Instructions
We use ```pyspacemouse``` to communicate with a connected spacemouse. In the conda environment run the following to install it,
```
pip install pyspacemouse
```
Please follow the full setup process at the [pyspacemouse repo](https://github.com/JakubAndrysek/pyspacemouse) if using spacemouse for the first time.

### Usage Instructions
For teleoperating the robot only using spacemouse, use ```telemoma/configs/only_spacemouse.py``` configuration.

Spacemouse has only 6-degrees of freedom, which is why TeleMoMa uses mode switching, and control each part of the robot independently. Upon startup it will control the right arm of the robot by default. Pressing the left side button of the spacemouse will rotate among controlling left arm, right arm, base and torso. The displacement of the spacemouse in each of the 6 degrees of freedom is tracked and sent as the delta commands to control the corresponding robot part. When controller the robot arms, pressing the right side button will toggle the gripper state between open and close.

## Keyboard
### Setup Instructions
We use ```pynput``` to communicate to read key presses. If you don't already have it installed then in the conda environment run,
```
pip install pynput
```
### Usage Instructions
For teleoperating the robot only using spacemouse, use ```telemoma/configs/only_keyboard.py``` configuration.

Keyboard presses are asynchronously read by the device listener and each key is mapped to a single DoF of the mobile manipulator. For a complete list of key mappings, see ```telemoma/input_interface/keyboard.py```. Note that each key increases / decreases one of the DoFs in the Cartesian space by some preset amount.
If they wish, users can also customize key bindings by directly changing the mapping in the script.

It is worth noting, however, that due to the large amount of keys mapped to various robot DoFs, it would be unintuitive to control the robot with only the keyboard. Therefore it is recommended to leverage the modularity of TeleMoMa and combine keyboard with other teleoperation methods. 

Note: If using keyboard control in iGibson, please deselect the viewer windows after launching the script. Since the iGibson viewers by default listen to keyboard strokes, this would help avoid unwanted behavior.

## iPhone
### Setup Instructions
We use the ARKit to track the phone's motion. Download the [app](https://testflight.apple.com/join/wGtbRk4w) on your phone and connect to the same network as the development computer. Set the IP of the develepment computer and the port to be used to listen to the mobile phone readings over [here](https://github.com/UT-Austin-RobIn/telemoma/blob/8ca898c306b4cd93faef82a0a30df755fe4f9b6b/telemoma/configs/only_mobile_phone.py#L4). 

### Usage Instructions
For controlling the robot only using an iPhone, use ```telemoma/configs/only_mobile_phone.py``` configuration.

Start the script on the develeopment computer and press the \<Connect\> button on the mobile app. The app will prompt for an IP and a port number, set it to be the same as earlier.

Once the phone is connected you can press \<Start\> and start controlling the robot. The \<Gripper\> button is overloaded and is used to toggle between opening and closing the robot gripper. Controlling the base motion using a mobile phone is under development.

Note: If the mapping between the phone and the robot seems off, restart the app and reconnect while holding the phone in the desired initial position.
