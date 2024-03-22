# Install F1TENTH Driver Stack
By the end you will have your VESC tuned and LiDAR connection set up.

Follow [these instructions](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/index.html). They involve 3 steps:

1. Configuring the VESC: The VESC - or electronic speed controller - is the brain of the car. You will need to update the firmware to contain more experimental features and import the motor XML configuration file from [here](https://drive.google.com/file/d/1-KiAh3hCROPZAPeOJtXWvfxKY35lhhTO/view?usp=sharing).

> [!NOTE]
> The most difficult part is tuning the PID controller (proportional-integral-derivative) to fit the needs of your car. The PID controller is a continuous feedback loop that adjusts the motor to the values you want.
The goal of this tuning is to make the motor accelerate to the desired RPM (eg. 8000) as sharply as possible, without overshooting or undershooting at the beginning, while maintaining the same RPM as smoothly as possible around the designated value. A good PID tune for 8000 RPM would be reaching 8000 RPM instantly without overshooting or undershooting at its peak and maintaining roughly 8000 rpm with a deviation of ±25-50 RPM.
Experiment with sample values until you find the ones that suit your car the best.

> [!CAUTION]
> DANGER: You will need to position the car on an elevated surface and connect the VESC to your laptop with a USB type B cable. This will allow you to directly program the VESC from your laptop while freely testing the RPM output, which means your wheels will start moving. If you don't put the car in the air (on top of some boxes that don't block the wheels) the car will just start running when you press start on the odometry data test and will rip the cable out of your laptop which MIGHT DAMAGE IT OR BREAK IT, OR WORSE BREAK YOUR CAR!

2. [Hokuyo Ethernet 10LX Ethernet Connection](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/firmware_hokuyo10.html#hokuyo-10lx-ethernet-connection-setup): You will need to connect your Lidar to the ECU with the ethernet cable on the Lidar so that it can feed its laser scan data to the ECU.

3. F1TENTH Driver Stack Setup (no container)

> [!WARNING]
> The 3 commands mentioned in the documentation should be replaced with the ones provided later:

```bash
$   rosdep update

$   rosdep install --from-paths src -i -y

$   colcon build
```

should be replaced with:

```bash
$   rosdep update --include-eol-distros

$   rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r

$   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Note that all of these commands should still be ran in the f1tenth_ws workspace folder, that is, in the folder where you are going to build the project with colcon (contains the src directory).

> [!NOTE]
> You might find it useful to include the following commands in the ~/.bashrc file. The contents of this file are ran every time you spawn a new instance of a bash terminal, which means you won't have to manually type them every time you open a new terminal.
Note that for these changes to take effect you will need to either restart all your terminals (that need these commands) or use "source .bashrc" in them, although I think it's best to just restart them.

```bash

$   source /opt/ros/foxy/setup.bash # replace foxy with your ROS2 version

$   source ~/f1tenth_ws/install/setup.bash # replace f1tenth_ws with the name/path of your workspace
```

Now, whenever you see those commands, you can skip them. Obviously, only if you have already included them in your .bashrc file...

There is no need to perform the instructions from [this point](https://f1tenth.readthedocs.io/en/foxy_test/autoware/intro.html#f1tenth-recordreplay-demo) onwards, as they are optional.