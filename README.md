# TeleMoMa: A Modular and Versatile Teleoperation System for Mobile Manipulation

<img src="assets/telemoma_architecture.png">

[Shivin Dass](https://shivindass.github.io/)<sup>1</sup>, [Wensi Ai](https://wensi-ai.github.io/)<sup>2</sup>, [Yuqian Jiang](https://yuqianjiang.us/)<sup>1</sup>, [Samik Singh]()<sup>1</sup>, [Jiaheng Hu](https://jiahenghu.github.io/)<sup>2</sup>, [Ruohan Zhang](https://ai.stanford.edu/~zharu/)<sup>1</sup>, [Peter Stone](https://www.cs.utexas.edu/~pstone/)<sup>1,3</sup>, [Ben Abbatematteo](https://babbatem.github.io/)<sup>1</sup>, [Roberto Martín-Martín](https://robertomartinmartin.com/)<sup>1</sup>

<sup>1</sup>The University of Texas at Austin, <sup>2</sup>Stanford University, <sup>3</sup>Sony AI

[[Paper]](https://arxiv.org/abs/2403.07869), [[Project Website]](https://robin-lab.cs.utexas.edu/telemoma-web/)

TeleMoMa is a teleoperation toolkit that enables intuitive teleoperation of high-DoF mobile manipulators. We provide a common pipeline that supports several teleoperation interfaces such as vision, VR, mobile phones and more, and enables mixing-and-matching between them. While we provide code for several mobile manipulators in sim and real, the versatility of TeleMoMa allows easy plug-and-play teleoperation of any mobile manipulator in general. 

## Setup  
### System Requirements
- Ubuntu 20.04
- Python 3.9+

Other system configurations may work, but we haven’t tested them extensively and may only be able to provide limited support.

Note: TeleMoMa can work with higher versions of python but if iGibson sim environment is required then please use python3.9.

### Installation

```
git clone https://github.com/UT-Austin-RobIn/telemoma.git
cd telemoma
conda create --name telemoma python==3.9
conda activate telemoma
pip install -e .
```

### Install iGibson (Optional)
We provide a demo script to test different teleoperation modes in the iGibson sim environment. iGibson can be installed by running,
```
(telemoma) > pip install igibson
```
In case of installation or runtime issues with iGibson, please checkout their exhaustive [documentation](https://stanfordvl.github.io/iGibson/) and [github](https://github.com/StanfordVL/iGibson).

## Usage

### Quickstart

To verify your installation the following code can be used to run TeleMoMa in a sandbox environment in iGibson.
```
(telemoma) > python telemoma/demo_igibson.py --robot tiago --teleop_config telemoma/configs/only_keyboard.py 
```

```--teleop_config``` defines the input modality to be used for controlling each part of the robot. Some example configurations are provided in [telemoma/configs](telemoma/configs/) folder.

Make sure the input hardware is correctly setup by following the instructions provided in [Human Interface](telemoma/human_interface/README.md).

### Human Interface
Hardware specific setup instructions for devices compatible with TeleMoMa are provided [here](telemoma/human_interface/README.md).

### Robot Interface
The details on the sim environment installations, real robot codebase and instructions on setting up TeleMoMa on your own robot are provided [here](telemoma/robot_interface/README.md).

## Citation
```
Coming Soon...
```
<!-- ```
@article{
    shah2023mutex,
    title=,
    author=,
    booktitle=,
    year={2024}
}
``` -->

### Acknowledgements: [Mentioned here](acknowledgements.md)