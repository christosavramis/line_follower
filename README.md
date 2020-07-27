## Table of Contents

- [Setup](#setup)
- [How to run](#how-to-run)
- [Workspace Structure](#workspace-structure)

---

### Setup

1. You need to have [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) installed.
2. Create [a new catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) if you don't have one already.
3. Clone this repository inside workspace/src: `git clone https://github.com/FineasRobotics/Line-Follower.git`

---

### How to run

1. Source the workspace(`source devel/setup.bash` or check our [ROS Basics](https://docs.google.com/document/d/1HTMq7Cwe4MZPlNUSJqRnfYy1TClEv3lscJfn8Ei_yrE/edit?usp=sharing) doc for more information).
2. Run: `roslaunch line_follower simulation.launch`.   
Tip: you can make gazebo open its GUI by changing gui argument inside gazebo_simulation.launch file to true.

---

### Workspace Structure

