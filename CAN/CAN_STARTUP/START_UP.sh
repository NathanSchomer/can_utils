#!/bin/sh
#George Slavin and Stephen Wolfe


#make the control directory and place the control code in it
if [ ! -d control ]
then
   mkdir control
   mv control.c ./control
   mv canHeader.h ./control
   mv i2cNew.h ./control
fi

#go to the control directory
cd control
#compile the control code
gcc control.c -o control.exe -lm -lpthread

cd ..
#make the CAN directory and move the CAN files into it
if [ ! -d CAN ]
then
   mkdir CAN
   mv CANread.c ./CAN
   mv CANwrite.c ./CAN
fi
cd CAN

#compile the necessary CAN files
gcc CANread.c -o CAN.read.exe
gcc CANwrite.c -o CANwrite.exe

if [ ! -d CAN_STARTUP ]
then
  cd ..
  mv CAN_STARTUP ./CAN
  cd ./CAN/CAN_STARTUP
fi

cd CAN_STARTUP
#run the CAN_START script the place the overlay
chmod 744 CAN_STARTUP.sh
./CAN_STARTUP.sh

#create .profile that will load CAN on connection
cd ~
echo /home/root/CAN/CAN_STARTUP/CAN_STARTUP.sh > .profile

