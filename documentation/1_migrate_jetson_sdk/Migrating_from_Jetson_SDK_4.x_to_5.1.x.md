# Migrating from Jetson SDK 4.x to 5.1.x

At the time of writing this the latest stable version of the Jetson SDK is version 5.1.3. 

> [!NOTE]
> **Applies to - Jetson Xavier NX (P3668-0000) only.** If you have a newer version of Xavier NX boards you can skip these steps (not tested!)

This documentation is only relevant in cases of upgrading the firmware from:
- Ubuntu 18.06 
- ROS1 
- Jetson SDK 4.x

to 
- Ubuntu 20.04  
- ROS2 distros foxy or galactic.
- Jetson SDK 5.x (Tested on 5.0.2, 5.1.1 and 5.1.3)

> [!WARNING]
> Ubuntu 22.04, Jetson SDK 6.x, and ROS2 distros humble or iron are NOT supported on older models of Xavier NX such as the one we used in this project. More recent ECUs like Jetson Orin support those.

Refer to [this video](https://www.youtube.com/watch?v=NJyHJzG6On0) or follow the instructions below. 

To upgrade the Jetson Xavier NX Developer Kit from JetPack 4.x to JetPack 5.x, complete the following steps: 

1. **Download the JetPack 5.x SDK** from the [NVIDIA Developer website](https://developer.nvidia.com/embedded/jetpack-sdk-513).

2. Write the QSPI image into the QSPI device by entering the following commands: 

```bash
$   sudo flash_eraseall /dev/mtd0

$   tar -xvf Jetson_Xavier_NX_QSPI_35.1.tbz2 jetson-xavier-nx-devkit.spi.img

$   sudo flashcp jetson-xavier-nx-devkit.spi.img /dev/mtd0
```

3. Power off the Jetson device and unplug the SD card.
4. Write the SD card with the JetPack 5.x SD card image by completing the steps in [Write Image to the microSD Card](https://developer.nvidia.com/embedded/learn/get-started-jetson-xavier-nx-devkit#write). (This step is proven by using jetpack versions 5.0.2, 5.1.1 and 5.1.3).

Complete the Ubuntu setup by selecting language, time zone, etc. Setup your username and password and you will have your jetpack 5 working! 