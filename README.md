# Introduction to F1TENTH - Newcastle University

Welcome to [F1TENTH](https://github.com/GeorgeTsakoumakis/f1tenth-ros2-newcastleuniversity), a part of Newcastle University's initiative inspired by the F1TENTH project. We would like to express our gratitude to the F1TENTH organization and contributors for their open-source efforts in the development of the F1TENTH platform.

## About F1TENTH
[F1TENTH](https://www.f1tenth.org/) is an open-source project that aims to provide an accessible and affordable platform for autonomous racing research. The platform is based on 1/10th scale RC cars and provides a simulation environment for testing algorithms and a physical platform for testing in the real world. The project is designed to be a community-driven initiative, with contributions from researchers, students, and hobbyists. Newcastle University's project attempts to modernise and standardise the efforts made by more recent contributions to the field and provide an up-to-date guide on the F1TENTH project. This project is licensed under the [GPL-3.0](https://www.bing.com/ck/a?!&&p=7349ec305c3df544JmltdHM9MTcxMTA2NTYwMCZpZ3VpZD0xMjFhZWYxNC03NjEwLTYyN2QtMjUxNS1mZGU0NzdkMDYzZTMmaW5zaWQ9NTI1NA&ptn=3&ver=2&hsh=3&fclid=121aef14-7610-627d-2515-fde477d063e3&psq=gpl+3&u=a1aHR0cHM6Ly93d3cuZ251Lm9yZy9saWNlbnNlcy9ncGwtMy4wLmVuLmh0bWw&ntb=1) License.

## Attribution

Our project builds upon the foundation laid by the F1TENTH community. We acknowlege the contributions of the F1TENTH organization and its contributors to the development of the autonomous racing platform. Specific attributions include:
- Project Name: F1TENTH
- License: MIT License

F1TENTH is a part of Newcastle University's initiative to modernise and standardise the efforts made by more recent contributions to the field. Any modifications or adaptations made to the F1TENTH project are specific to our project and licensed under the GPL-3.0 License.

## How to contribute

If you are interested in contributing to the F1TENTH project, we encourage you to visit their official site and get involved in the vibrant community. If you wish to contribute to Newcastle University's initiative, please make sure you have obtained permission by the organization or from its maintainers.
We extend our appreciation to the F1TENTH community for fostering an environment of collaboration and knowledge sharing.

# How to run

First make sure you have the following prerequisites:
- Fully-built F1TENTH car
- ROS2 and dependencies installed (follow F1TENTH documentation)
- [f1tenth_system](https://github.com/f1tenth/f1tenth_system) repo cloned and configured (odometry calibartion, etc.). This is crucial as you cannot run any autonomous algorithms without the `bringup_launch.py` file. To make sure you have this on your car navigate to your workspace and check that `f1tenth_system` is present in the `src` folder.

To install the project, follow these steps:

If the code is not present on the car (for Newcastle University cars you can probably skip this step):

1. Clone the repository to your workspace
    - ```cd /path/to/your/workspace/src```
    - ```git clone https://github.com/GeorgeTsakoumakis/f1tenth-ros2-newcastleuniversity.git```
2. Navigate back to worksace (parent of src)
    - ```cd ..``` , alternatively to be safe `cd /path/to/your/workspace`
3. Build the workspace
    - ```colcon build``` 

To run the project:

1. Source the workspace
    - ```source /path/to/your/workspace/install/setup.bash```
2. Source ROS2
    - ```source /opt/ros/foxy/setup.bash``` (or whichever ROS2 version you are using)

> [!tip]
> The two steps mentioned can be added into your `.bashrc` file to avoid sourcing every time you open a new terminal. Also, you can alias the following commands in your .bashrc file to avoid typing them every time you want to run the project:

```alias manual_drive="ros2 launch f1tenth_system bringup_launch.py"```

```alias follow_the_gap="ros2 launch follow_the_gap follow_the_gap"```

For more information on how these commands work learn about node creation in ROS2 and how to launch them.

3. Launch manual drive
    - ```ros2 launch f1tenth_system bringup_launch.py``` (or use the alias if you have set it up)

4. In another terminal, launch the algorithm
    - ```ros2 run follow_the_gap follow_the_gap``` (or use the alias if you have set it up). This step is the same for any other algorithm you want to run.

## Camera feed

To view the camera feed, you can either run in a terminal `realsense-viewer` and enable the RGB option, or you can run the following command followed by `rviz2` in another terminal:

```ros2 launch realsense2_camera rs_launch.py```
(for Newcastle Uni cars 1 and 2 the alias should be start_cam - check .bashrc file)

Once rviz is open, add a new Camera or Image display, select the topic `/camera/color/image_raw` and you should be able to see the camera feed. Also at the top replace `map` with `camera_link`.

> [!CAUTION]
> As of right now, the camera CANNOT be powered sufficiently using the battery. The problem is most likely happening with the old powerboard model, which is why it is recommended to try only after upgrading to the newest F1TENTH powerboard model (purple). The camera however works perfectly fine when the car is connected to the power supply.

# Maintainers

## Authors
- [Georgios Tsakoumakis](https://github.com/GeorgeTsakoumakis) - [g.tsakoumakis2@newcastle.ac.uk](mailto:g.tsakoumakis2@ncl.ac.uk)
- Mayuresh Inamdar - [m.s.inamdar2@newcastle.ac.uk](mailto:m.s.inamdar2@ncl.ac.uk)

## Supervisors
- [Ken Pierce](https://github.com/kgpierce) - [kenneth.pierce@newcastle.ac.uk](mailto:kenneth.pierce@newcastle.ac.uk)
- [Paulius Stankaitis](https://github.com/pastankaitis) - [Paulius.Stankaitis@newcastle.ac.uk](mailto:Pualius.Stankaitis@newcastle.ac.uk)

## Maintainers

- [Georgios Tsakoumakis](https://github.com/GeorgeTsakoumakis) - [g.tsakoumakis2@newcastle.ac.uk](mailto:g.tsakoumakis2@ncl.ac.uk)
