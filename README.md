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
pip install -r requirements.txt
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
@article{dass2024telemoma,
  title={TeleMoMa: A Modular and Versatile Teleoperation System for Mobile Manipulation},
  author={Dass, Shivin and Ai, Wensi and Jiang, Yuqian and Singh, Samik and Hu, Jiaheng and Zhang, Ruohan and Stone, Peter and Abbatematteo, Ben and Martín-Martín, Roberto},
  journal={arXiv preprint arXiv:2403.07869},
  year={2024}
}
```

### Acknowledgements: [Mentioned here](acknowledgements.md)



## My Stuff
- Checkout `telemoma-real` branch
- Missing: `conda run -n ${ENV_NAME} pip install -e .`
- Maybe run `xhost +local:docker` on host machine (outside docker) to allow GUIs to open
- Realsense in docker: `https://github.com/2b-t/realsense-ros2-docker`
  - Command to test camera: `realsense-viewer`
- Telemoma realsense instructions: `https://github.com/UT-Austin-RobIn/telemoma/blob/telemoma-real/telemoma/human_interface/README.md`
  - ROS node - DON'T RUN SIMULTANEOUSLY TO PYTHON CODE, otherwise python can't access the camera: `source /opt/ros/noetic/setup.bash && roscore & && roslaunch realsense2_camera rs_camera.launch`
  - `conda activate telemoma && pip install -e . && source /opt/ros/noetic/setup.bash`
- `python telemoma/demo_real.py --robot hsr --teleop_config telemoma/configs/only_vision.py`


### VR
Installation
1. Don't install ros-noetic-desktop-full, but only the basic version, otherwise will break VR. ros-noetic-tmc-desktop-full will also break it!
2. `sudo apt install ros-noetic-hsrb-common ros-noetic-hsrb-interface-py ros-noetic-tmc-common-msgs`
3. DON'T do this one: `ros-noetic-hsrb-robot`
4. conda create -n telemoma python=3.11
5. install robot_io: https://github.com/dHonerkamp/robot_io/blob/main/docs/teleoperation.md
6. install telemoma dependencies `pip install -e .`
7. install real-robot telemoma dependecies, see real-robots.md / Dockerfile
8. FMM: DON't instlal `ros-noetic-franka-ros`
9. `sudo apt install ros-noetic-ridgeback-description ros-noetic-realsense2-description ros-noetic-arbotix-msgs ros-noetic-catkin python3-catkin-tools`
ros-noetic-franka-description ros-noetic-franka-msgs ros-noetic-controller-interface ros-noetic-controller-manager
  ros-noetic-controller-manager-msgs ros-noetic-franka-gripper ros-noetic-franka-hw ros-noetic-franka-example-controllers
10. `mkdir -p catkin_ws_franka/src && cd src && git clone git@rlgit.informatik.uni-freiburg.de:fmm/rl_franka.git && source devel/setup/bash`
11. `pip install spatialmath-python`

1. Start SteamVR and make sure all is connected. In a new place, first to the room-setup thing it prompts you to do
2. `bullet_vr`
3. run script

FMM:
1. unlock brakes
2. activate FCI
3. BASE COMPUTER: 
```
roslaunch fmm amcl.launch &
roslaunch rl_franka fmm_franka_control.launch
```
4. USER COMPUTER: 
`rosrun map_server map_server /home/honerkam/repos/mobile-rl/real_world2/map.yaml`
5. This computer / terminal running from: `sudo route add -net 192.168.131.0 netmask 255.255.255.0 gw 192.168.0.40` otherwise rosservice calls to controller_manager will fail
6. `source catkin_ws_franka/devel/setup.bash && rviz -d /home/honerkam/repos/telemoma/rviz.rviz`


### IMBIT Network
1. Added to DHCP in Router Config (Archer in browser)
2. Changed `/etc/netplan/00-installer-config.yaml` -> backup in `sudo vim /etc/netplan/00-installer-config.yaml.building080`
```
# This is the network config written by 'subiquity'
network:
  ethernets:
    eno1:
    dhcp4: yes
      addresses:
              #- 10.8.105.204/24
      - 192.168.0.205/24
      gateway4: 192.168.0.1
      nameservers:
        addresses:
        - 8.8.8.8
        - 8.8.4.4
          #search:
          #- informatik.privat
          #- imbit.privat
          #- informatik.uni-freiburg.de
  version: 2
```
3. `sudo netplan apply`
4. Add `192.168.0.40    fmm-4` to `/etc/hosts`



