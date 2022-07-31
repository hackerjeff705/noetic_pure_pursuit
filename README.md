# Pure Pursuit
Pure pursuit implementation on ROS noetic.
For the theory overview and code breakdown visit the medium article below.
<https://medium.com/@jefffer705/pure-pursuit-in-ros-noetic-7b2c0a3c36ef>

## Algorithm
1. Get current vehicle position (x, y, w, v)
2. Find nearest waypoint to vehicles current position.
3. Find the waypoint (goal point) closest to the lookahead.
4. Calculate alpha between the nearest waypoint and the goal point.
5. Calculate the curvature and steering angle.
6. Update the vehicles position, steering angle, velocity etc.
