# RoboBehaviors-and-FSM
CompRobo 2025 - Ben Ricket, Connor Hoang

## Overview
This warmup project serves as an introduction to ROS2 and working with the Neatos with various tasks. We implemented the following five behaviors for this project: wall following, wall detection, wall approaching, turning, and stopping.
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
When the wall following node is initialized, the robot should first identify the nearest wall to it, which is not guaranteed to be in front of it. In this state, the robot uses its LIDAR scan data to detect the nearest wall and rotate until it is facing that wall or an equally close wall, before transitioning into an `approach` state. 
#### Implementation
This is achieved by accessing the entire contents of the last LIDAR scan, rather than sampling a specified range. To avoid sensor noise indicating a wall/obstacle is closer than it is in reality, `WALL_MIN_PTS` consecutive LIDAR scan ranges are checked simultaneously, and the maximum of these distances taken to be the distance to the wall in that direction. Of these distances, the minimum distance to an obstacle across the entire `ranges` array is used as the distance to a closest obstacle. 

After identifying this distance, the robot turns by calling the `turn` command, and checks if the difference between the distance to an obstacle in front of the robot and the minimum distance identified by the LIDAR scan is less than a specified tolerance (`WALL_SEARCH_TOLERANCE`). If this is the case, the robot is effectively facing the nearest wall, and can progress to approach.

Transitions: If the difference between the distance to a wall in front of the robot and the minimum distance to an obstacle is less than `WALL_SEARCH_TOLERANCE`, change state to `approach`.

A subscription to the `/bump` topic and corresponding callback changes the state to `bump` if the bump sensor ever outputs nonzero values, and sends a zero-velocity command to stop the robot. 

Issues: By using the maximum distance between `WALL_MIN_PTS` consecutive LIDAR readings as its metric of how close obstacles are in that direction, this implementation is effectively blind to any obstacles that, from the perspective of the LIDAR scanner, occupy less angular distance than the angle between the *n*th and the (n + WALL_MIN_PTS)th LIDAR scan rays. As we don't expect objects this thin to regularly occur or pose an issue to the robot, we consciously make this tradeoff, though `WALL_MIN_PTS` could be tuned to adjust this.

Additionally, we currently only occupy this state after initializing the node, and are unable to return to it. Permitting the robot to return to this state can result in repeated behavior where the robot loses track of a wall on the right \(for instance, it drives past an object on the right, no longer detecting it\), has no idea of where a wall is, and turns back to the object, follows it briefly, and loses it again, becoming trapped in this behavior.

### Wall Approach
#### Logic
In the approach state, the Neato is attempting to close the distance between itself and the wall directly in front of it. It will slow down as is approaches the wall.
#### Implementation
This is acheived by setting an initial linear velocity ```Kp_drive``` that is multiplied by the difference between the distance in front of the Neato and the target distance (```approach_vel = self.Kp_drive * (dist_front - self.target_distance)```). The effect is a linear velocity that decreases proportionaly as it approaches the wall. This approach velocity is bound to determined max and min velocites (```min(self.max_vel,max(self.min_vel,approach_vel))```) and sent to drive. Drive is a helper function that takes in arguments for linear and angular velocities which are then sent to the Neato.

Transitions: If distance in front of the Neato is less than `target_distance` change to state `turn`.

A subscription to the `/bump` topic and corresponding callback changes the state to `bump` if the bump sensor ever outputs nonzero values, and sends a zero-velocity command to stop the robot. 

### Turn
#### Logic
In the `turn` state, the Neato recognizes that there is an obstacle or wall in front of it, and turns counterclockwise until the path ahead is clear. If the path is clear and a wall is still present to the right of the Neato, it can move to the `wall_follow` state, following the wall to the right; otherwise, it recognizes it has lost the wall, and returns to `approach` in order to find the next wall.
#### Implementation
This is achieved by setting an angular velocity for turning, `self.max_ang_vel`, which is in turn set by the ROS2 parameter `max_ang_vel`, and passing this angular velocity to a `drive()` function that instructs the Neato to turn. In the current implementation, the turn is only counterclockwise --- while the `turn()` function called in this state takes a boolean argument for the direction of the turn, this not actively used, as knowing whether a right or left turn is ideal requires tracking the distance from a wall both to the right and left of the robot, which we do not currently do. 

Transitions: After every iteration of the `turn` behavior, if the distance to the nearest wall in the front is greater than the target distance from a wall, then we assume the path in front of the robot is clear. At this point, if the distance to the nearest wall to the right of the robot is less than our `identify_wall_distance`, the distance at which we register a wall to our right, we register the wall to the right and transition to `wall_follow` in order to follow that wall. If the distance on the right is greater than `identify_wall_distance`, we have lost our wall, and return to `approach` in order to find another wall. 

A subscription to the `/bump` topic and corresponding callback changes the state to `bump` if the bump sensor ever outputs nonzero values, and sends a zero-velocity command to stop the robot. 

### Wall Follow
#### Logic
The Neato is attempting to drive parallel to the wall directly to its right.
#### Implementation
The ```wall_follow``` case starts by calculating the linear velocity of the Neato is always set to a constant velocity as defined by the parameter ```self.declare_parameter("max_vel",0.2)```.

Transitions:

A subscription to the `/bump` topic and corresponding callback changes the state to `bump` if the bump sensor ever outputs nonzero values, and sends a zero-velocity command to stop the robot.

### Bump
#### Logic
This state only exists to hold the robot in place when the bump sensor publishes nonzero values to `/bump`. 

#### Implementation
A subscription to the `/bump` topic and corresponding callback changes the state to `bump` if the bump sensor ever outputs nonzero values, and sends a zero-velocity command to stop the robot. There are no transitions from this state --- the script must be stopped and run again to resume operation of the robot. Ideally, the robot should not enter this state if the controls and parameters are tuned properly, though in practice this is not always the case. 

## Conclusion

### Takeaways
### Challenges
One of the challenges we faced throughout the project was tuning paramters. Specifically, the angle of the search cone that determined the distances from the robot and the coefficents in the wall following PID logic. 

Another challenge we encountered was how our initial state change and wall follow logic made no distinction between `target_distance`, the setpoint for our wall following PID controlle, and the distance at which we registered a wall to the right of the robot as "detected" for the purpose of choosing to follow it or trying to find a new wall. This check was necessary to ensure the robot would not follow a wall while being far away from it, but by having this parameter be the same as the wall follow setpoint, any deviation away from the wall could trigger the `wall_follow` --> `approach` transition, rather than remaining in `wall_follow` and letting the PID controller handle the deviation. To address this behavior, we had to add an additional `identify_wall_distance` parameter, indicating a distance at which we recognize and intend to follow a wall even if it is greater than our ideal target distance. 

As a minor aside, structuring the code as a single ROS2 node effectively led to it being contained within a single Python class, relegating it to a single file, which just made concurrent development a bit more annoying. This could have been avoided by writing some of the helper functions outside of the class itself, or using multiple ROS2 nodes, but the code for this project was short enough to not make this much of an issue. 

### Additional Documentaion
Video and bag files are included in the repo as validation for the stated behaviors.

### Next Steps
Were we to develop this project further, the current code could benefit from an automated pipeline to tune the wall follow PID parameters. To achieve this, we could develop a Gazebo simulation to use specifically to test the wall follow performance, create a launch file initializing this simulation with the wall_follow behavior, and develop a metric to automatically measure the performance of the wall follow behavior and tune the parameters accordingly. 