# Quickstart

This guide will get you setup with Little Red Rover, and should take less than an hour. In the end you'll have an autonomous robot running ROS's [NAV2 tools](). 

## Unboxing / Assembly

Included in the box you'll find the following:

 * One circuit board
 * One LD-20 lidar
 * Two orange wheels
 * Two motors with encoders
 * One 18650 LiIon battery, 5000 mah
 * One 4-pin JST GH 1.25mm straight across cable
 * Two 6-pin JST PH 2.0mm reversed cables
 * An assortment of 3D printed parts
    - One front skid
    - Two motor holders
    - Two motor spacing shims
    - Two motor attachment shims
    - Two wheel spacers
 * Three M3-8 machine screws
 * One M2.5 hex wrench
 * One USB-C to C charging cable

With these parts in hand, you're ready to assembly your robot.

TODO: Assembly video

## Software

### Dependencies

Little Red Rover uses Docker as a compatability layer with your host operating system ([what is docker?]()).
The steps for installing Docker differ depending on your host OS.

TODO: Docker installation for Mac, windows, linux, ect

For a new user, Docker can have a steep learning curve. Luckily, the [DevContainers]() project allows for seamless integration within your editor.
VSCode has first class support for DevContainers, and is the recommended editor for LRR.

TODO: VsCode installation for Mac, windows, linux, ect

Once VSCode is installed, use the extension manager to install the DevContainers extension.

TODO: How to install extension

Lastly, LRR uses Git / Github for source code management. To download the source code, you must first install git. 

TODO: How to install Git

### Setting Up Your Development Environment

Our first step is to download the LRR source. 

TODO

Next, we need to open the project in VSCode.

TODO

Finally, open the project using the DevContainers extension. This first installation will take some time (you're downloading all of Ubunutu and ROS2 after all).
It's recommended to use a strong internet connection during this initial setup.

TODO

Congrats! Your development enviroment is now setup.

### Connection to the Robot

On first setup, and whenever you take your robot to a new location, a network connection needs to be configured.

First, power on your robot. To do so, insert your battery and move the power switch (located just next to the USB port) to the on position.


### Rock and Roll

Now that the robot is setup, lets have some fun. To start a demo, run the following commands in your VSCode shell:

To build the ROS2 project:

bash
```
lrr_build
```

To run the demo:

bash
```
lrr_run
```

Your robot is now running, but we have no way to see whats going on!
For that, we use a program called [Foxglove]().

## Caring for your robot

### Battery Charging and Care

LRR comes with a rechargable 5000 MAh lithium ion battery. To recharge the battery, attach the provided USB cable between the robot and a power source.
LRR contains two red status LEDs for power, located next to the USB-C port.
The first, labeled `PWR GOOD`, turns on when the USB-C connection is supplying ample power to the system.
The second, labeled `CHARGING`, turns on when the battery is actively charging. Once your battery is fully charged, this light will turn off.

> [!WARNING]
> NEVER, ever ever ever, charge lithium batteries unattended.
> Both LRR and the provided battery contain protection circuitry, but there is still a NON-ZERO CHANCE THAT THE BATTERY WILL CATCH FIRE.
> This issue is intrinsic to all cells with high power density, and it is your responsibility to use them safely.

### Storage and Transport

Little Red Rover's packaging is carefully designed to protect your robot once its assembled. Remove and lose foam and place the robot back into the box for transport.

## Troubleshooting

firewall port
