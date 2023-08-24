## Read the wiki pages
This readme is only a small overview of the actual use of the repository. Use the wiki for more helpful information.

## Overview
This repository is used by the ISU duckietown team for use with the motion planning class. It was forked from [the duckietown ros-template repository](https://github.com/duckietown/template-ros)

## Pure Pursuit Library
Pure Pursuit is an algorithm that can follow a given trajectory. It depends on $l$ the lookahead distance to determine how closesly it can follow a given path. $l$ is the lookahead distane, i.e, the point to which the robot is going to navigate. Think of it like when a human drives, we look a certain distance ahead of the vehicle.

- PurePursuit
    - This is the main class. 