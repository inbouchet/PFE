# PFE
## Use of turtlebot

This file describes how to launch the turtlebot to reproduce the presented project.

### Turtlebot side
```
roslaunch turtlebot3_bringup turtlebot3_robot.py
```

### Master side
```
roscore
```

If you want to test the Aruco detection with the Robot :
```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch pfe controller_tb.launch
```

If you want to test de communication with the Drone :
```
export TURTLEBOT3_MODEL=waffle_pi
rosrun pfe listener.py
rosrun pfe move_goal.py
```

If you want to test the odometry of the Robot :
```
export TURTLEBOT3_MODEL=waffle_pi
rosrun pfe odometry.py
```

