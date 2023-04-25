# LaTTe: LAnguage Trajectory TransformEr

<video width="100%" controls>
  <source src="./docs/media/ICRA2023_LaTTe_low.mp4" type="video/mp4"/>
</video>
<!-- ![iterative NL interactions over a trajectory](./docs/media/interactions.gif)
-->


## setup
<sub>_tested on Ubuntu 18.04 and 20.04_</sup>

[install anaconda](https://docs.anaconda.com/anaconda/install/linux/)

Environment setup
```
conda create --name py38 --file spec-file.txt python=3.8
conda activate py38
```
Install CLIP + opencv
```
pip install ftfy regex tqdm dqrobotics rospkg similaritymeasures
pip install git+https://github.com/openai/CLIP.git
pip install opencv-python
```


Download models

```
pip install gdown
gdown --folder https://drive.google.com/drive/folders/1HQNwHlQUOPMnbPE-3wKpIb6GMBz5eqDg?usp=sharing -O models/.
```
Download synthetic dataset  
```
gdown --folder https://drive.google.com/drive/folders/1_bhWWa9upUWwUs7ln8jaWG_bYxtxuOCt?usp=sharing -O data/.
```

Download image dataset(optional)
```
gdown --folder https://drive.google.com/drive/folders/1Pok_sU_cK3RXZEpMfJb6SQIcCUfBjJhh?usp=sharing -O image_data/.
```


## Running the visual demo

```
cd src
python interactive.py
```

**How to use:**

1) press 'o' to load the original trajectory
2) press 'm' to modify the trajectory using our model for the given input on top.
3) press 't' to set a different interaction text.
4) press 'u' to update the trajctory setting the modified traj as the original one

intructions for additional keyboard commands are shown in script output.

---
## ROS setup:

> **IMPORTANT:** make sure that conda isn't initialized in your .bashrc file, otherwise, you might face conflicts between the python versions 

[install ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

<!-- [manually install CVbridge](https://cyaninfinite.com/ros-cv-bridge-with-python-3/)
> **NOTE:** this is the catkin config that I used to intall CVbridge with the Anaconda </br>
```catkin config -DPYTHON_EXECUTABLE=$CONDA_PREFIX/bin/python -DPYTHON_INCLUDE_DIR=$CONDA_PREFIX/include/python3.8 -DPYTHON_LIBRARY=$CONDA_PREFIX/lib/libpython3.8.so -DSETUPTOOLS_DEB_LAYOUT=OFF``` -->

For realtime object detection:
```
git clone https://github.com/arthurfenderbucker/realsense_3d_detector.git
```

## Running with ROS
Install the darknet_ros msgs from https://github.com/ugupta62/darknet_ros <br>
In terminal 1
```
roscore
```
terminal 2
```
roscd latte/src
python interactive.py --ros true
```

---

## PX4 + ROS + GAZEBO
Setup PX4 simulator <br>
https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html#simulation-and-nuttx-pixhawk-targets <br>

Download MAVROS<BR>
https://docs.px4.io/main/en/ros/mavros_installation.html#binary-installation-debian-ubuntu <br>

Goto mavros directory using `roscd mavros/launch` and replace the `px4.launch` file with the launch file given in `latte/mavros/px4.launch`. Then launch the file by:
```
roslaunch mavros px4.launch
```

Setup Gazebo <br>
https://docs.px4.io/v1.12/en/simulation/gazebo.html <br>

Type`make px4_sitl gazebo__warehouse` in the PX4-Autopilot directory, change a parameter in the px4 console inorder to make the quadrotor loiter at its current position when the offboard mode is unavailable by:
```
param set COM_OBL_RC_ACT 5
```
You will see the connection between mavros and PX4 gazebo simulator in the terminal where you launched the px4.launch file as follows:

```
[ INFO] [1681691712.238305338]: IMU: Attitude quaternion IMU detected!
[ INFO] [1681691712.562083623]: VER: 1.1: Capabilities         0x000000000000ecff
[ INFO] [1681691712.562151256]: VER: 1.1: Flight software:     010e0080 (5c1e0ddd96000000)
[ INFO] [1681691712.562189673]: VER: 1.1: Middleware software: 010e0080 (5c1e0ddd96000000)
[ INFO] [1681691712.562207569]: VER: 1.1: OS software:         050e00ff (0000000000000000)

```
In the px4-gazebo terminal, type `commander takeoff` for taking off and run the trajectory_tracking.py file. Then in the same terminal type `commander mode offboard` to engage the offboard command. In order to engage the trajectory tracking, in a new terminal, type `rosparam set /is_navigation_started True`.

## Other relevant files
overview of the project
[model_overview.ipynb](model_overview.ipynb)

model variations and ablasion studies
[Results.ipynb](Results.ipynb)

user study interface
[user_study.py](user_study.ipynb)

generate syntetic dataset
[src/data_generator_script.py](src/data_generator_script.py)


