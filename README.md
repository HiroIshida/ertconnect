## ERT connect
ERT-connect is one of the newest experience based planner [1]. ERT connet implementation exists as a PR to a OMPL [https://github.com/ompl/ompl/pull/783](https://github.com/ompl/ompl/pull/783). However the PR seems inactive. 

So, This repository pickup the PR and port to a standalone repository and provide minimalistic example.

In the original code in PR, the procedure to pickup a single trajectory from the library is not implemented. However, from the equation (4) of the RA-L paper, I found that it is the same as Lightning's. Thus, I implemented the relevant trajectory selection by using LightningDB already exists in ompl.


## Install
Assume you are running on ubuntu 20.04 or higher (will fail to build in older ubuntu due to boost version mismatch)

install and build
```bash
sudo apt install libompl-dev libboost-filesystem-dev libboost-serialization-dev -y
git clone https://github.com/HiroIshida/ertconnect
cd ertconnect
mkdir build && cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release ..
make -j4
```

## example
The problem setup is quite simple: squared 2dim world with circle obstacle. To measure performance I conducted experiment, where trajectory library is constructed with 1000 trajectories using RRT-connect. Then solving ERT-connect for 300 time for different problem  with static environment. 
```bash
./build/example 
```
The output is however shows the computation time of rrt-connect and ert-connect is almost the same. 
```
rrt-connect average solving time: 0.0257019
ert-connect average solvign time: 0.0251004
```
I think, the parameter tuning is required, or, probably my usage of the code is wrong.  Anyway, I should try and explorer more. 

## TODO
add comparison with lightning [2] and thunder [3] planners.

## reference
[1] Pairet, Èric, et al. "Path planning for manipulation using experience-driven random trees." IEEE Robotics and Automation Letters 6.2 (2021): 3295-3302.
[2] Berenson, Dmitry, Pieter Abbeel, and Ken Goldberg. "A robot path planning framework that learns from experience." 2012 IEEE International Conference on Robotics and Automation. IEEE, 2012.
[3] Coleman, David, et al. "Experience-based planning with sparse roadmap spanners." 2015 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2015.


