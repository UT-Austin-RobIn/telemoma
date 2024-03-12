# TeleMoMa: A Modular and Versatile Teleoperation System for Mobile Manipulation

<img src="assets/telemoma_architecture.png">

[Shivin Dass](https://shivindass.github.io/)<sup>1</sup>, [Wensi Ai](https://wensi-ai.github.io/)<sup>2</sup>, [Yuqian Jiang](https://yuqianjiang.us/)<sup>1</sup>, [Samik Singh]()<sup>1</sup>, [Jiaheng Hu](https://jiahenghu.github.io/)<sup>2</sup>, [Ruohan Zhang](https://ai.stanford.edu/~zharu/)<sup>1</sup>, [Peter Stone](https://www.cs.utexas.edu/~pstone/)<sup>1,3</sup>, [Ben Abbatematteo](https://babbatem.github.io/)<sup>1</sup>, [Roberto Martín-Martín](https://robertomartinmartin.com/)<sup>1</sup>

<sup>1</sup>The University of Texas at Austin, <sup>2</sup>Stanford University, <sup>3</sup>Sony AI

[[Paper]](), [[Project Website]](https://robin-lab.cs.utexas.edu/telemoma/)

## Setup  
### System Requirements
- Ubuntu 20.04
- Python 3.9+

Other system configurations may work, but we haven’t tested them extensively and may only be able to provide limited support.

Note: TeleMoMa can work with higher versions of python but if iGibson sim environment is required then please use python3.9.

### Installation

```
git clone https://github.com/ShivinDass/telemoma.git
cd telemoma
conda create --name telemoma python==3.9
conda activate telemoma
pip install -r requirements.txt
pip install -e .
```

## Usage

### Quickstart
To verify your installation the following code can be used to run TeleMoMa in a sandbox environment in iGibson.
```
(telemoma) > python telemoma/demo_igibson.py --robot tiago --teleop_config telemoma/configs/only_vision.py 
```

```--teleop_config``` defines the input modality to be used for control each part of the robot. Some example configurations are provided in [telemoma/configs](telemoma/configs/) folder.

Make sure the input hardware is correctly setup by following the instructions provided in [Human Interface](telemoma/human_interface/README.md).

### Human Interface
Hardware specific setup instructions for devices compatible with TeleMoMa are provided [here](telemoma/human_interface/README.md).

### Robot Interface
The details on the sim environment installations, real robot codebase and instructions on setting up TeleMoMa on your own robot are provided [here](telemoma/robot_interface/README.md).

## Citation
```
TODO...
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