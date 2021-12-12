# robot velocity filters
Set of tools for filtering desires twist of the EE, as well as joint velocities with a second order dynamics.


## Installation
Do the following steps:
* In your catkin src directory clone the repository
```
$ git clone https://github.com/nbfigueroa/robot_velocity_filters.git
```
* wstool gets all other git repository dependencies, after the following steps you should see extra catkin 
  packages in your src directory.
```
$  wstool init
$  wstool merge robot_velocity_filters/dependencies.rosinstall 
$  wstool up 
```
* Query and installs all libraries and packages 
```
$ rosdep install --from-paths . --ignore-src --rosdistro <your-ros-distro> 
```


## Usage

---
## Contact
Maintainer: [Nadia Figueroa](https://nbfigueroa.github.io/) (nadiafig @ mit dot edu)

## License
- This code is released under MIT license. 
