# opel-can-swc-adapter
Opel CAN-BUS adapter based on Arduino, CAN shield and Microchip digital potentiometer to use steering wheel, generate ignition, illumination and reverse signal for aftermarket headunits.

CAN code is based on https://github.com/vtl/volvo-kenwood
Steering Wheel code is based on https://github.com/bigevtaylor/arduino-swc

The code is tested on Opel Corsa-D and should work for almost all Opel cars with MS-CAN.
Used headunit is Pioneer SPH-DA230DAB

Prototype was build on Arduino NANO, MCP2515 CAN module and MCP4131 digital potentiometer

Version 0.0 
First release and test in the car. Timing for button press and detecting of long press needs to be modified.

04.12.2020 - Added schematic
