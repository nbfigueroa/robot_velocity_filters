## online_motion_generator
---
This package provides tools for online motion generation of smooth cartesian and joint-space trajectories for a N-DOF robot.
...


## Installation
Do the following steps:
* In your catkin src directory clone the repository
```
$ git clone -b nadia https://github.com/nbfigueroa/online_motion_generator.git
```
* wstool gets all other git repository dependencies, after the following steps you should see extra catkin 
  packages in your src directory.
```
$  wstool init
$  wstool merge online_motion_generator/dependencies.rosinstall 
$  wstool up 
```
* Query and installs all libraries and packages 
```
$ rosdep install --from-paths . --ignore-src --rosdistro indigo 
```


## Usage


**Contact**: [Nadia Figueroa](https://nbfigueroa.github.io/) (nadiafig AT mit dot edu)
