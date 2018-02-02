# Evaluating-Human-Hand-Pose
This project implementes human hand pose estimation model and evaluation loss function. It is based on ros and includes one package named my_turtle/. There are three parts for the total project: Depth image process, forward kinematic hand model and loss function. The first two parts can be found in forward_Chain.cpp, loss function can be found in lossfunc.cpp 


## Project description:
+ Image process: transform depth image into 3D points cloud.
+ Forward kinematic hand model: build humanoid hand model using KDL chain and DH parameters.
+ Evaluation: evaluate the hand pose estimation model using loss function.

## Commands to run the code:
Before running the code please load relative parameters

```sh
For depth image:
rosparam load param.yaml

For static hand pose:
rosparam load hand_param.yaml

To run the node:
source devel/setup.bash
rosrun using_markers forward_Chain
```
