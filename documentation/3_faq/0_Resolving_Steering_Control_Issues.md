# Resolving Steering Control Issues

## Issue 1: Loose Servo Connections

Identify the issue:

- Notice erratic or non-responsive steering control.
- Inspect the physical connections of the servo motor.

Check connections
 
- Ensure that the servo motor connections are securely plugged into the appropriate ports on the F1TENTH car's VESC board.
- If connections are loose, firmly plug them in to establish a reliable electrical connection. Make sure your USB cable is not damaged/bent.

Test Steering Control:

- Verify that the steering control now functions smoothly and responsively.

## Issue 2: Incorrect Joystick Mapping for Steering Control

Locate the configuration files:

- Open the joy_teleop.yaml file within the `f1tenth_system` repository.
- Path: `/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/joy_teleop.yaml`

Adjust the `scale` parameter:

- Modify the `scale` parameter in the `joy_teleop.yaml` file to adjust the sensitivity of the steering control.

```
drive_steering_angle:
    axis: <axis_number>
    scale: <new_scale_value> # Adjust the scale value to fine-tune steering sensitivity
    offset: 0.0
```
(By default, `scale` will be set to 0.34. Change it to 1.0 and check if you are getting any response)

- Save the changes to the file, navigate to the `f1tenth_ws` directory, and rebuild the workspace using `colcon build` and test again.

- If that still doesn't work, then change the axis value from default 2 to axis: 3.

- Sometimes joystick buttons are mapped differently. For the Right joystick to work change the *<axis_number>* with the appropriate axis for the right joystick.

- Save the changes to the file, go back to the `f1tenth_ws` directory, and rebuild the workspace using `colcon build` and test again.

## Issue 3: Enabling Servo Output in VESC Tool

Identify the issue:

- Users may encounter difficulties steering the F1TENTH car due to not enabling servo output in the VESC Tool.

Connect VESC:

- Launch the VESC Tool application on your computer.
- Ensure that your VESC is properly connected to your **host machine** via USB or another supported interface.
- Verify that the VESC Tool detects the connected VESC. If not, troubleshoot the connection (E.g., check the USB cable, restart VESC Tool).

Navigate to the App Settings Tab:

- In the VESC Tool interface, navigate to the "App Settings" tab on the left and click on General.
- Change the setting of *Enable Servo Output* to True.

![VESC tool](https://files.gitbook.com/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fua1w77Pt2UjpW7aSOCDn%2Fuploads%2FBQbHhFZso0DZlZxcSfxS%2Fdelete.png?alt=media&token=ccf06ef7-575f-48dc-9fad-f97b9963f85d)

- Once that is done apply the setting by clicking on the *Write App Configuration* button on the right-side panel.

![VESC tool](https://files.gitbook.com/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fua1w77Pt2UjpW7aSOCDn%2Fuploads%2FhvNm6VKuazKInFpCbSVj%2Fdelete.png?alt=media&token=619f7257-1d4f-4d06-9274-19aa09ec94bf)