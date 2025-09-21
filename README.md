# RoboBehaviors-and-FSM
CompRobo 2025 - Ben Ricket, Connor Hoang

## Overview
This warmup project serves as an introduction to ROS2 and working with the Neatos with various tasks. We implemented the following __ behaviors for this project: wall following, __, and ____.
### List of Behaviors
1. K
2. l
3. l
## Behaviors Implemented
### Overall Code Structure
The code for this project is a single python file ```wall_follow.py``` structured into a singular ```NeatoFsm``` class that extends the ROS2 node. Inside the primary run loop of the node is a switch case that that explicitly checks for "approach", "wall_follow", ___, and ___. Excluding the initial state which is ```wall find```, each case first calls a method to execute the code related to the state and then checks if any of the transition modes are met (i.e. in approach the Neato is instructed to have a certain linear velocity and then checks if it is within the desired closeness to the wall).

We chose this structure for its___. 
### Wall Find
#### Logic
#### Implementation

### Wall Approach
#### Logic
In the approach state, the Neato is attempting to close the distance between itself and the wall directly in front of it. It will slow down as is approaches the wall.
#### Implementation
This is acheived by setting an initial linear velocity ```Kp_drive``` that is multiplied by the difference between the distance in front of the Neato and the target distance (```approach_vel = self.Kp_drive * (dist_front - self.target_distance)```). The effect is a linear velocity that decreases proportionaly as it approaches the wall. This approach velocity is bound to determined max and min velocites (```min(self.max_vel,max(self.min_vel,approach_vel))```) and sent to drive. Drive is a helper function that takes in arguments for linear and angular velocities which are then sent to the Neato.

Transitions: If distance in front of the Neato is less than target distance, change to state "Turn."

### Turn
#### Logic
#### Implementation

### Wall Follow
#### Logic
The Neato is attempting to drive parralel to the wall directly to its right.
#### Implementation
The ```"wall_follow``` case starts by calculating The linear velocity of the Neato is always set to a constant velocity as defined by the parameter ```self.declare_parameter("max_vel",0.2)```.
## Conclusion

### Takeaways
### Challenges
One of the challenges we faced throughout the project was tuning paramters. Specifically, the angle of the search cone that determined the distances from the robot and the coefficents in the wall following PID logic. 
### Additional Documentaion
Video and bag files are included in the repo as validation for the stated behaviors.
### Next Steps
