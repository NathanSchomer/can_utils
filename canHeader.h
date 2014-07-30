#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
 
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
 
#include <linux/can.h>
#include <linux/can/raw.h>

//Device IDs
//#define TOGGLE_MOTORS 5
#define BB_ID 0
#define MOTOR1_ID 1
#define MOTOR2_ID 2
#define MOTOR3_ID 3


//COMMAND INTERFACE
#define ECHO 0
#define MODE 1
#define SET_GAIN 2
#define SET_VELOCITY 3

//PID param IDs
#define SET_P 1
#define SET_I 2
#define SET_D 3

//This union can be used to convert between different types
//stores 64 bits of information
//use the char array when passing this union to the canSend function
typedef union
{
  char c[8];
  int i[2];
  float f[2];
  double d;
  unsigned long long ull;
  unsigned long int uli[2];
  signed long int li[2];
  struct {
    unsigned char b1:1;
    unsigned char b2:1;
    unsigned char b3:1;
    unsigned char reserved:5;
  } bits;
  struct{
    float f;
    int i;
  } floatInt;
} canData;

///////////////////////////////////////////////////////////////////////////////////////////
//CAN sending functions
//canSend:
//command -> "number Id of the command to send determined by motor board code"
//src -> "source ID (BeagleBone ID is 0)"
//dest -> "ID of the destintation board (motor boards have IDs 1 to 3)"
//data -> "the payload of whatover command is being sent (always a pointer to a char array)
//  use the union canData to properly package the data, then give canSend the char array in the union"
//length -> "length in bytes of the data.  Always 4 right now.  (see data above)"
int canSend(int command, int src, int dest, char * data, int length);

//setCanFrame:
//"fills in a can_frame struct and returns a pinter to the can_frame struct
// This function is called by canSend.
// See above function canSend for argument description."
struct can_frame *setCanFrame(int command, int src, int dest, char * data, int length);

//CAN recieving functions
//canRecieve:
//numberOfMessages -> "the number of CAN messages that will be read from the bus before the function stops reading"
int canRecieve(int numberOfMessages);

//printfCANFrame:
//"Print recieved CAN message to the stdout"
//can_frame -> "a struct defined by SocketCAN use setCanFrame function to define your CAN frames"
int printCANFrame(struct can_frame frame);


///////////////////////////////////////////////////////////////////////////////////
//Control functions:
//each sends a CAN message to the motor board
//using the appropriate CAN message

//echo:
//"echos back the message it was send"
//motor -> "motor ID number"
void echo(int motor);

//toggleMotor:
//"Enables and disables the motors"
//on ==1 -> "turn motor on"
// on ==0 -> "turn motor off"
void toggleMotor(int motor, int on);

//setMotorVelocity:
//motor -> "motor ID number"
//velocity -> "velocity value to pass to motor"
void setMotorVelocity(int motor, signed long int velocity);

//setP, setI, set D:
//"set the parameters for the PID controller on the motor boards"
//motor -> "motor ID number"
//param -> "the parameter to change. P=1,I=2,D=3"
//value -> "value to set the PID parameter to"
void setGain(int motor,int param, float value);