This repository contains two methods for controlling a ethercat based Delta Robot.

BOTH:
This contains shared source files. (used by SOEM and TwinCAT) They are written in C.

SOEM:
This implementation runs on linux and windows. As such it can work on a beaglebone/raspberry pi. Written in C.

TwinCAT:
This implementation uses the official Beckhoff software. Only works on windows and windows embedded. Written in c++.

Hardware Datasheets:
Contains datasheets for motors/magnets/etc. plus ethercat slaves.

Electrical Configuration:
Contains information about how ethercat slaves are connected to actuators. This is the same configuration that was used with the original NXT software described below.

OldConfiguration:
Contains information about how the system was configured when it was used with the NXT Control software. (This software is not in this repository. It is too large and does not work). This software was used to control the robot initially, however it seems a proprietary firmware image became corrupt and the company cannot find a suitable copy as such, this control method no longer works.
