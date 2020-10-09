## Line Follower Camera Vision
[![Watch the video](https://img.youtube.com/vi/E4BY5rPSpE0/hqdefault.jpg)](https://www.youtube.com/watch?v=E4BY5rPSpE0)

## Bot Completes The Track
[![Watch the video](https://img.youtube.com/vi/vdwuJwgXQyE/hqdefault.jpg)](https://www.youtube.com/watch?v=vdwuJwgXQyE)


## Table of Contents

- [Setup](#setup)
- [How to run](#how-to-run)
- [Control the robot with the keyboard](#control-the-robot-with-the-keyboard)

--- Table of Contents

- [Setup](#setup)
- [How to run](#how-to-run)
- [Control the robot with the keyboard](#control-the-robot-with-the-keyboard)

---

### Setup

1. You need to have [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) installed.
2. Create [a new catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) if you don't have one already.
3. Clone this repository inside workspace/src: `https://github.com/FineasRobotics/line_follower.git`
4. Install packages: `sudo apt-get install ros-melodic-turtlebot3 ros-melodic-turtlebot3-msgs ros-melodic-turtlebot3-simulations`
5. Run from the root of workspace: `catkin_make`
6. To use turtlebot3, you need to specify which model you are going to use. To do that:
    - You either run `export TURTLEBOT3_MODEL=${TB3_MODEL}` where `${TB3_MODEL}` you put burger, waffle or waffle_pi before running the simulation.
    - Or you put the above line at the end of your ~/.bashrc, with: `gedit ~/.bashrc`
**PS**: We use waffle.

---

### How to run

1. Source the workspace(`source devel/setup.bash` or check our [ROS Basics](https://docs.google.com/document/d/1HTMq7Cwe4MZPlNUSJqRnfYy1TClEv3lscJfn8Ei_yrE/edit?usp=sharing) doc for more information).

#### In simulation
2. Run: `roslaunch line_follower simulation.launch mode:=sim`.   
Tip: you can make gazebo open its GUI by changing gui argument inside gazebo_simulation.launch file to true.

#### With a real robot
2. Run: `roslaunch line_follower simulation.launch`.   

---

#### Control the robot with the keyboard

Open another terminal and run: `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch` 

---

### How to run tests
1. To test line_follower, run inside test folder: `python -m unittest test_line_follower`
