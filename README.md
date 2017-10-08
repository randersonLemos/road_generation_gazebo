# Ros package dedicated to generate a circuit (corridor of obstacles) in gazebo from a sequence of points.
---
## Overview 
With the defined launche files it is possible to generate and see the .worlds files of a circuit defined by obstacles. Two kinds of obstables are already avaiable which are a cyllinder and a tree. The central circuit is defined by a sequence of points like [..., (xi,yi), ...]. This circuit, that can be open or close, is used as reference to create two others circuits that are parallel to the reference one (the central circuit) and that are used to guide the placement process of obstacles in such way that in the end, globally, the placed obstacles form a corridor. 

## Usage
1. Use the **world_generation.launch** file to generate the .worlds file. In this files it is possible do set several parameters to generate the corridor of obstacles as desired. The central circuit  waypoints are defined in the file **road_metadata** which is located at yaml folder. This launch file will generate a **custom.world** file that are saved in the folder worlds.

2. To see the generated corridor of obstacles at Gazebo use the **show_world.launch** file passing the name of the world, custom.world, to the parameter world_name.

3. The package comes with a **example.world** file with an illustrative corrigor of obstacles that can be visualized launcing the **show_world.launch** without passing arguments.