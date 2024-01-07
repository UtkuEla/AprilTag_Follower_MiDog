# CNN Supported Robust Fiducial Markers Detection And Following Using Quadruped Robot

[![Made with ROS](https://img.shields.io/badge/Made%20with-ROS-green?&logo=ros)](http://wiki.ros.org/)
[![Python 3.8](https://img.shields.io/badge/Python-3.8-3776AB?logo=python)](https://www.python.org/downloads/release/python-360/)
[![Gazebo](https://img.shields.io/badge/GAZEBO-orange?logo=gazebo&logoColor=white)](https://gazebosim.org/home)
[![React](https://img.shields.io/badge/REACT-blue?logo=react&logoColor=white)](https://reactjs.org/)

|                                                                    Bittle Model                                                                     |                                             Apriltags                                              |
| :-------------------------------------------------------------------------------------------------------------------------------------------------: | :------------------------------------------------------------------------------------------------: |
| ![](https://hackster.imgix.net/uploads/attachments/1350269/hackster-front_O66b4x4vua.gif?auto=format%2Ccompress&gifq=35&w=900&h=675&fit=min&fm=mp4) | ![](https://cdn.shopify.com/s/files/1/0292/0693/7678/files/apriltag-pad_1_grande.png?v=1594511445) |

# Getting Started

# 1- Implementation

> **_NOTE:_** <br />
> 1- This Robot model urdf is cloned from this repo, [Bittle_URDF](https://github.com/AIWintermuteAI/Bittle_URDF) <br /> 2- while the JointEffortService and the mixer implemenation has been cloned from this repo [notspot_sim_py](https://github.com/lnotspotl/notspot_sim_py)

<p align="center">
<img  src="./assets/follow.gif" />
</p>

<br />

1. Clone this project.

   -

   ```bash
   $ git clone SSH or HTTPS
   ```

2. ## Go to the workspace:
   ```bash
   $ cd Final Code Base
   ```
3. Edit main configuration variables in settings.yaml if required:

   -

   ```bash
   remoteControl, aprilTagLocalization, etc...
   ```

4. ## run final version file:
   ```bash
   $ python AprilTagFollowerFinal_v4.py
   ```

# CNN Supported Version:

1. Go to directory.

   -

   ```bash
   $ cd construction-robotics-ws-2022_23/CNN
   ```

2. ## Test model:
   ```bash
   $ python followerwithCNN.py
   $ python AprilTagFollower_withCNNsupport.py
   ```
