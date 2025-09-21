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
### Wall Search
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
In the `turn` state, the Neato recognizes that there is an obstacle or wall in front of it, and turns counterclockwise until the path ahead is clear. If the path is clear and a wall is still present to the right of the Neato, it can move to the `wall_follow` state, following the wall to the right; otherwise, it recognizes it has lost the wall, and returns to `approach` in order to find the next wall.
#### Implementation
This is achieved by setting an angular velocity for turning, `self.max_ang_vel`, which is in turn set by the ROS2 parameter `max_ang_vel`, and passing this angular velocity to a `drive()` function that instructs the Neato to turn. In the current implementation, the turn is only counterclockwise --- while the `turn()` function called in this state takes a boolean argument for the direction of the turn, this not actively used, as knowing whether a right or left turn is ideal requires tracking the distance from a wall both to the right and left of the robot, which we do not currently do. 

Transitions: After every iteration of the `turn` behavior, if the distance to the nearest wall in the front is greater than the target distance from a wall, then we assume the path in front of the robot is clear. At this point, if the distance to the nearest wall to the right of the robot is less than our `identify_wall_distance`, the distance at which we register a wall to our right, we register the wall to the right and transition to `wall_follow` in order to follow that wall. If the distance on the right is greater than `identify_wall_distance`, we have lost our wall, and return to `approach` in order to find another wall. 

### Wall Follow
#### Logic
The Neato is attempting to drive parralel to the wall directly to its right.
#### Implementation
The ```"wall_follow``` case starts by calculating The linear velocity of the Neato is always set to a constant velocity as defined by the parameter ```self.declare_parameter("max_vel",0.2)```.
## Conclusion

### Takeaways
### Challenges
One of the challenges we faced throughout the project was tuning paramters. Specifically, the angle of the search cone that determined the distances from the robot and the coefficents in the wall following PID logic. 

Another challenge we encountered was how our initial state change and wall follow logic made no distinction between `target_distance`, the setpoint for our wall following PID controlle, and the distance at which we registered a wall to the right of the robot as "detected" for the purpose of choosing to follow it or trying to find a new wall. This check was necessary to ensure the robot would not follow a wall while being far away from it, but by having this parameter be the same as the wall follow setpoint, any deviation away from the wall could trigger the `wall_follow` --> `approach` transition, rather than remaining in `wall_follow` and letting the PID controller handle the deviation. To address this behavior, we had to add an additional `identify_wall_distance` parameter, indicating a distance at which we recognize and intend to follow a wall even if it is greater than our ideal target distance. 
### Additional Documentaion
Video and bag files are included in the repo as validation for the stated behaviors.
### Next Steps
