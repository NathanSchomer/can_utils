The Ball Bot Project
George Slavin and Stephen Wolfe

To get started run the START_UP.sh script

This script sets up the CAN1 bus (refered to as can0 by Angstrom) by setting up a device tree overlay.
IF THIS SCRIPT THROWS AN ERROR
OPEN THE START_UP.SH SCRIPT AND EDIT THE FOLLOWING LINE

/sys/devices/bone_capemgr.*/slots 

CHANGE THE * TO THE CORRECT NUMBER
you can find the correct number using the following command:
cd /sys/devices/
look in that directory for bone_capemgr


this also compiles three files:
i2cNew
CANread
CANwrite


Discription of compiled files:

i2cNew:
This is the main balance program.  Start i2cNew.exe and the Ball bot will be controllable by the imu.  Tilt the IMU in a direction and the ball will correct in the opposite direction.  The functions used in this file are detailed in the two header files.


CANread:
This function recieves CAN messages and prints them to stdout.
Run in the background like this:
	./CANread.exe 1000 &
1000 is the number of can messages to recieve before the program ends

CANwrite:
This function sends out CAN messages.  use in this format:

	./CANwrite.exe [command number] [source ID] [destination ID] [payload] [number of bytes]

The beaglebone ID is 0 and the motors IDs are 1 through 3
To broadcast to all three motors use 7 which is the broadcast address.

example (send command 1 to motor 3 with an argument of 1000 using all 8 bytes):
	./CANwrite.exe 1 0 3 1000 8

Here is the current list of commands supported by the boards:
ID   COMMAND       ARGS
0    Echo          no arguments(just send 0 for the payload)
1    LED           send 1 to turn on. Send 0 to turn off
2    Motor Toggle  send 1 to enable motor.  Send 0 to disable motor 
3    Set Velocity  send the number of ticks/s for the mtoor to turn

Device ID list
ID   DEVICE
0    Beaglebone Black
1    Motor 1
2    Motor 2
3    Motor 3
