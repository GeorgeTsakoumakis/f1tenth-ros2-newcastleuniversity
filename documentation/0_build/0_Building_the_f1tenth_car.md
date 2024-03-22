# Building the F1TENTH car

You will find a bill of materials in [this link](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/build_car/bom.html) provided by F1TENTH. Note that prices and currencies might be different for your project. It is estimated that you will need around £1700-2000 to complete the build, without accounting for labor costs. 

> [!NOTE]
> The antenna mount needs to be 3D printed according to [these specs](https://drive.google.com/drive/folders/1sy-XiJJ4hmhEKf5qQbUaPYY6Aw-L31Gk). Also, you will need to laser cut the platform deck according to [these specs](https://drive.google.com/drive/folders/1NU4FZzvMEGKCOFzDBvnjyePnnSMvsZPG).

> [!NOTE]
> The Jetson Xavier NX ECU board has been EOL (end-of-life) since April 2020. As of the date of this documentation (February 2024), we are experimenting with replacing it with the Jetson Orin ECU, which is still maintained. However, this documentation will focus on the Xavier NX, until further updates.

> [!IMPORTANT]
> Make sure you have flat and Philips screwdrivers and hex keys (M2, M3).

> [!WARNING]
> The F1TENTH Autonomous Vehicle uses lithium polymer batteries. LiPO batteries allow your car to run for a long time, but they are not something to play with or joke about. They store a large amount of energy in a small space and can damage your car and cause a fire if used improperly. With this in mind, here are some safety tips for using them with the car.
- When charging batteries, always monitor them and place them in a fireproof bag on a non-flammable surface clear of any other objects.
- Do not leave a LIPO battery connected to the car when you’re not using it. The battery will discharge and its voltage will drop to a level too low to charge it safely again.
- Unplug the battery from the car immediately if you notice any popping sounds, bloating of the battery, burning smell, or smoke.
- Never short the battery leads.
- Do not plug the battery in backwards. This will damage the VESC and power board (and likely the Jetson as well) and could cause a short circuit.
- See [​this video](https://www.youtube.com/watch?v=gz3hCqjk4yc) for an example of what might happen if you don’t take care of your batteries. Be safe and don’t let these happen to you!