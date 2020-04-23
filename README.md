# Whereisthis
code for "Where is this" in RoboCup Shaoxing 2020

## Introduction

The task takes place at "home" (home-like arena, with rooms and some furniture). Robots are required to "*explain and show people where they can find places in the arena (e.g. Where is the TV?)*". The robot has to tell people the route to get there and then take them there "*in a type of tour guide*" (which means the robot introduce and points at the object as it passes by).

Before the task starts, the robot stays at a predefined location (**"start point"**), waiting for referee to tell it where it should go (**"information point"**) (e.g. information point is "near the dinning table", a predefined location).

After the information point is told to the robot, the referee will open the "door" (a board) for the robot. The robot finds the door open, and enters the arena, going directly to information point.

Then an operator stands in front of the robot, asking for the location of one object (TV, desk, etc.). The robot describes the route, and take the operator to the destination. It should introduce and point at the objects it passes by (e.g. saying "you can see a chair on your right side" as well as pointing at the chair).

Finally it goes back to information point waiting for the next operator. The process is cycled 6 times (3 operators, 2 times per operator). The robot is encouraged to interact with people **with previous knowledge** when it sees the same operator for a second time ,i.e. it remebers where the operator went before. 

## Process

divide into several sub-tasks:

### Before the game

* **Mapping**: Get the map of arena as accurate as possible using g-mapping or RRT.
  * **Related Section(s)**: navigation

* **ObjectLocating**: Get the location of every object/place from the map as accurate as possible.
  * **Related Section(s)**: navigation

* **CreateRoute**: Establish the route description from one object/place to another.
  * **Related Section(s)**: speech

### During the game

* **GetPoint**: recognize where the information point is.
  * **Related Sections**: speech (publisher), navigation (subscriber)

* **DoorOpening**: detect the door in front of the robot using laser.
  * **Related Section(s)**: vision/image

* **GotoPoint**: robot guides to information point.
  * **Related Section(s)**: navigation

* **GoalDescription**: robot ask the operator to stand in front of the camera -> robot recognize the operator's face -> operator asks question -> robot describes the route. **ATTENTION**: if robot sees the same operator for a second time, it should also describe the route from previous destination to current destination.
  * **Related Section(s)**: vision/image (publisher), speech (subscriber)

* **Guiding**: robot guides to destination. **ATTENTION**: if robot is close enough to another object/place (not information point/destination), the robot should find out whether the object/place is at its left side or at its right side (we don't consider other conditions temporarily), and says "you can see a (name) at your right/left side" with arm pointing at left or right side.
  * **Related Section(s)**: navigation (publisher), speech (subscriber), arm (subscriber)

* **BacktoPoint**: robot guides to information point, waiting for the next operator to ask question (next time cycle from GoalDescription).
  * **Related Section(s)**: navigation

## On-going work

1. Better robustness at navigation: when one door is opened, the robot needs to find another available way quickly.
2. Introduce object in front of the robot. relatively easy to implement.
3. Design and implementation of Emergency Control Flow Part.
4. Better roboustness at speech: too sensitive to voice now. may be mistakenly waken up (wake-up word: Jack); high rate of false recognition.
5. (updated 2020-4-24) Bugs: cannot go back to information point; lots of annoying warnings; tour-guide introducing haven't been tested; image section only tested in Kinect (not astra); cannot leave the arena automatically

## Use

The structure of 'Whereisthis' is similar to a previous respository of mine 'Receptionist' with one control center and four functional sections.

We use turtlebot2 for the competition, with camera (both Kinect and astra), turtlebot arm and laser. Make sure you have Ubuntu 16.04 and python2.
The following package is needed while not listed in this respository:
**speech, pockectphinx, xfei_asr**
However, these can be found from the previous respository 'Receptionist'. (wish i didn't lost some)
The work of mapping and marking objects is also needed but not listed here, since it is nearly the same as that in Receptionist, with only a few changes.
Run this command to set up map:

```ROS
roslaunch reptionist_navigation receptionist_map_setup.launch
```
Control the robot to move to every corner of the site using keyboard. 

Run this command to save mark points:
```ROS
roslaunch receptionist_navigation receptionist_get_waypoints.launch
```
Similarly, you let the robot move to the point at a specific pose, then you type 'get' in a small window (you will see it). Type 'stop' when finishing all the points. The data is stored in 'waypoints.txt', and you should copy the data to 'where\_nav.py' at the function 'init_position'.

The description of route should be written to 'where\_is_this.py' at the function 'generate\_description'.

Run this command to start:
```ROS
roslaunch whereisthis_control whereisthis_whole.launch
```
**Remeber to mark the initial pose of the robot!!!** At Rviz, click '2D Pose Estimation', and mark the pose in the map. 
It is better to move the robot to better be assure that the initial pose is accurate enough (which can be automatic but seems like we don't have time). The easiest way is to move the robot using keyboard:
roslaunch turtlebot_teleop keyboard\_teleop.launch
After all of this, now you can say 'Jack', and tell the robot where the information point is. GOOD LUCK!

e-mail: jinwei_nankai@foxmail.com  jinwei\_nankai@163.com
