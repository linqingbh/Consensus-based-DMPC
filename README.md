# Consensus-based-DMPC
A leader-follow formation algorithm for fixed-wing UAVs.

Simulation code for the paper in matlab.


## Paper

[Consensus-based Control of Multiple Fixed-wing UAVs Using Distributed Model Predictive Control](https://ieeexplore.ieee.org/document/9336925)

If you find this code useful in your work, please consider citing:

H. Liu, J. Hu, C. Zhao, X. Hou, Z. Xu and Q. Pan, "Consensus-based Control of Multiple Fixed-Wing UAVs Using Distributed Model Predictive Control," 2020 7th International Conference on Information, Cybernetics, and Computational Social Systems (ICCSS), Guangzhou, China, 2020, pp. 858-863.

## introduction 

In this paper, a consensus-based distributed model predictive control algorithm for fixed-wing UAVs is proposed.

There is a new framework to combine consensus algorithm
of formation control with DMPC. By using this, all fixed
wing UAVs could achieve desired formaiton simultaneously
with collision avoidance. And the formation will not be
changed in the next time period. The algorithm can be
separated into two parts. The former part is consensus
algorithm which all followers compute its trajectories by
using states errors compared to leader. The latter part
is DMPC algorithm which each agent using its tracking
trajectory and information from neighbors to get optimal
input of next movement.

## example

### Matlab show

![avatar](gif/2.gif)

![avatar](gif/1.gif)



### UE4
We also tried the Unreal enignee 4 for better performance.

![avatar](gif/3.gif)

![avatar](gif/4.gif)

## How to run

Just run the file "main".

"main" is a conbination of the "CalAlluavs" and "ShowAlluavs"

Several uavs data are provided in folder "data", you can change some code in "main" to use them by following the rules.

If you want to compute your own data, "A5pre" is provided in the folder "Formation_control_main".

## Other-links

[DMPC-for-multi-agent](https://github.com/chengji253/DMPC-for-multi-agent)


[DMPC-cpp](https://github.com/chengji253/DMPC-Cpp)
