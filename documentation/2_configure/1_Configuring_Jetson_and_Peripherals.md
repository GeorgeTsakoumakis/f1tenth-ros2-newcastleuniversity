# Configuring Jetson and Peripherals 

[This link](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/software_setup/index.html) of the official F1TENTH documentation refers to setting up the following:

- Page 1: This involves installing the Jetson SDK on our ECU, setting up the boot sequence from the SSD (note: SD card will still be necessary), creating a swapfile for memory-intensive tasks, and installing the logitech joystick drivers

- Page 2: This page gives instructions on how to connect to your car via SSH wirelessly. For that, you will need to have installed the antennas (obviously, as you will need WiFi connection) and have a device (hopefully laptop) that can connect on the same network to the car via SSH. 

> [!NOTE]
> Using remote desktop as a protocol is not advised, because of increased latency and the instability of the RDP protocol. 99% of the time SSH is the way to go, so don't overcomplicate it.

> [!WARNING]
> If you are working from a workspace at Newcastle University you will not be able to use the university WiFi, as the network configuration will most likely prohibit you from pinging the car on the network.

> [!TIP]
> A workaround you can use is to use your own personal hotspot from your phone. The network operations are not very intensive (especially after setting up the car) and you will not have to consume too much data. An alternative - which is discouraged - would be to set up your own network with your own router, but good luck with getting that approved through IT... However, all this is not an issue if you are working on a WiFi that does not have this issue.