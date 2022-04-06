---
layout: default
title: Robot Construction
parent: Setup
nav_order: 3
---

# Robot Setup

This document outlines the steps to construct the two-wheeled inverted pendulum robot. Before starting the construction you will several things. First, consult the [BOM](setup.md) for the parts required for the robot. Second, open up the [CAD](https://cad.onshape.com/documents/430d4af740243dc0e842d2a6/w/42cfe57eb2477e6d1bc0bfd3/e/dd30410687349c102bbbacfb) and obtain DXF files of the robot frame (base plate, main wall, side walls, etc). At this point you can also obtain the STL file for the battery holder which you can 3D print.

## Robot Frame
To construct the frame you will need access to a laser cutter. Import the DXF files obtained from the CAD into your favourite laser cutting software and cut the pieces using 5mm hardboard. The support legs do not have to be cut from hardboard, we found that they tended to break often, rather they can be waterjet cut or cut with a stronger material.


<img src="../../resources/images/IMG_2527.png" alt="laser cut software" width="200"/>

Example of the laser cutter software.

<img src="../../resources/images/IMG_2526.png" alt="laser cut material" width="200"/>

Cutting the hardboard.

Once the material has been cut, epoxy together the frame. The order that the board should be put together is as follows
- The left, right and center wall
- The top and bottom plates
- The handle (optional)

<img src="../../resources/images/IMG_frame_epoxy.png" alt="epoxy step 1" width="200"/>
<img src="../../resources/images/IMG_frame_epoxy_2.png" alt="epoxy step 2" width="200"/>

<img src="../../resources/images/IMG_2530.png" alt="final frame" width="200"/>

The frame should look similar to the image above.

## Assembly
Once the frame has been assembled, you are ready to put all the electronics together. To get started, gather some tools.

<img src="../../resources/images/IMG_2529.png" alt="assembly tools" width="200"/>

### Terminal Block
The first item to mount will be the terminal block. Cut the jumpers into groups of three and insert them into the block. Using some M3 bolts, mount the block to the side opposite where the jetson will eventually sit.

<img src="../../resources/images/IMG_2531.png" alt="terminal block" width="200"/>
<img src="../../resources/images/IMG_2532.png" alt="terminal block mounted" width="200"/>

### Motor Driver
Mount the motor drivers on the same side as the terminal blocks. Ensure you have a pair of wires which you will connect to the terminal block later.

<img src="../../resources/images/IMG_2533.png" alt="motor driver" width="200"/>
<img src="../../resources/images/IMG_2534.png" alt="motor driver mounted" width="200"/>

### Central Electronics Board
On the opposite side of the middle wall, you will mount the main PCB that wire mounts are connected to. The regulator should be placed on the same side as the terminal block.

<img src="../../resources/images/IMG_2535.png" alt="pcb" width="200"/>
<img src="../../resources/images/IMG_2536.png" alt="pcb mounted" width="200"/>
<img src="../../resources/images/IMG_2537.png" alt="pcb mounted" width="200"/>

### Battery Holders
3D print the battery holders with supports using your favourite 3D printer. Insert the fuse holder, battery connecter and switch into their slots and solder some connections as shown.

<img src="../../resources/images/IMG_3D_batt_holder.png" alt="batt holder" width="200"/>

<img src="../../resources/images/IMG_3D_batt_holder_2.png" alt="batt holder" width="200"/>
<img src="../../resources/images/IMG_3D_batt_holder_3.png" alt="batt holder" width="200"/>
<img src="../../resources/images/IMG_2538.png" alt="batt holder" width="200"/>
<img src="../../resources/images/IMG_2539.png" alt="batt holder" width="200"/>
<img src="../../resources/images/IMG_2540.png" alt="batt holder" width="200"/>

### Motor & Mount
Mount the motor onto the L-bracket mount using some M3 bolts. Insert the wheel onto the motor shaft and secure it using a M3 set screw.

<img src="../../resources/images/IMG_2541.png" alt="motor and wheel" width="200"/>

### Jetson & Rest
Finally, mount the Jetson on the same side as the central PCB. Place the Jetson near the side to ensure access. At this point you can optionally add the support legs (first remove the terminal block to get access) using M5 bolts.

<img src="../../resources/images/IMG_final.png" alt="final" width="200"/>

### Wiring
The terminal block is used as a central location where the motor drivers, regulator, batteries can connect to. The connections in the block should have the following groups

1. Regulator Power in, Battery (A) Power
2. Regulator Ground, Battery (A) Ground, MPU6050 unused pins
3. Left Motor Driver Power, Right Motor Driver Power, Battery (B) Power
3. Left Motor Driver Ground, Right Motor Driver Ground, Battery (B) Ground]

The rest of the wires should be held down by zip ties to ensure they are secure.
