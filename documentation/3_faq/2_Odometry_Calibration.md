# Odometry Calibration

Original documentation can be found [here](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/driving/drive_calib_odom.html).

## Some intuition...

Tuning the **steering gain** is a very tricky and time-consuming process. The most important things to keep in mind are:
- Having a turning radius of about 1.78m
- Turning radius should be symmetrical from both sides.

It is important to note that changing this value also changes the scale of the inputs from the joystick. That is, if your joystick values go from 0.3 to 0.85 before changing the gain, they might go to 0.2 to 0.9 (increased scale). These changes are very hard to predict and require constant readjustments. They also lead to asymmetrical radii sometimes.

This is why you are going to need to adjust the `servo_min` and `servo_max` values - they are the lower and upper bounds for the input values. Changing them can help you "clip" them - you will see on the terminal when running the `bringup_launch.py` a message saying the value has been clipped to the minimum or maximum (left/right steering of the joystick).

Ultimately, the goal is to get the behaviour described above. Steering offset is pretty easy to adjust - simply adjust until you drive in a straight line. ERPM gain is also fairly simple, but consult the following.

## Corrections to official documentation

When tuning the ERPM gain, the following is mentioned in the documentation:

> If the distance reported by echo is larger, decrease the `speed_to_erpm_gain` value. Otherwise increase the gain.

It is actually the opposite way around. It should in fact say:

> If the distance reported by echo is larger, *increase* the `speed_to_erpm_gain` value. Otherwise *decrease* the gain.

Also:

> The change is usually on the order of thousands.

This is also wrong. It is in the order of hundreds.

In our case, we had to decrease the gain from 4800 to 4500 (I think) in order to increase the reported distance.