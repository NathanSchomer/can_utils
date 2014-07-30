//modified from code from http://www.frank-buss.de/io-expander/linux-i2c-test.c
//get orientation from Free-IMU
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <math.h>
#include <string.h>
#include <sys/time.h>
#include <sys/resource.h>

#define PI 3.1415926535f
#define CONTROL_FREQUENCY 200.0f 
#define TIME_STEP 0.05f
#define ACCEL_GAIN 1.0f
#define MAX_VELOCITY 1000.0f
//max number of motor counts per second allowed
#define MAX_CPS 50000.0f

////////////////////////////////////////////////////////////////////////////
//physical dimensions


//#define GEAR_RATIO = (CLUSTER_GEAR / PINION) * (OUTPUT_PULLEY/CLUSTER_PULLEY) * (D_BALL/D_WHEEL);

#define D_WHEEL         2.5f       //diameter of omniwheel
#define D_BALL         10.5f       //diameter of basketball
#define THETA          45.0f       //included angle between omniwheel's axis and vertical

#define PINION         16.0f       //motor pinion teeth
#define CLUSTER_GEAR   50.0f       //cluster gear teeth
#define CLUSTER_PULLEY 18.0f       //cluster pully teeth
#define OUTPUT_PULLEY  32.0f       //output pully teeth

#define ENCODER_CPR    2048.0f     //motor enconder counts per revolutions

////////////////////////////////////////////////////////////////////////////
//device addresses
//"This defines the device addresses as they appear on the I2C bus"
#define MAG_ADDR 0x1e
#define ACCEL_ADDR 0x19
#define GYRO_ADDR 0x6b

#define FIFO_FILE "SOCKET_FIFO"

////////////////////////////////////////////////////////////////////////////
//calibrations

//Accel Cal
//set full scale range to 2 g 
#define LA_FS 2
//get sensitivity with current full scale setting (mg/LSB)
#if LA_FS == 2
    #define So_A 1.0f
#elif LA_FS == 4
    #define So_A 2.0f
#elif LA_FS == 8
    #define So_A 4.0f
#elif LA_FS == 16
    #define So_A 12.0f
#endif

//Mag Cal
#define CRB_REG_M 0x80
//get sensitivity(M_GN and M_GNZ) and full scale setting(M_FS) 
//units:(LSB/guass and guass respectively)
#define GNbits 4
#if GNbits == 1 
    #define M_FS 1.3f
    #define M_GN 1100.0f
    #define M_GNZ 980.0f
#elif GNbits== 2
    #define M_FS 1.9f
    #define M_GN 855
    #define M_GNZ 760
#elif GNbitsS == 3
    #define M_FS 2.5f
    #define M_GN 670.0f
    #define M_GNZ 6000.f
#elif GNbits == 4
    #define M_FS 4.0f
    #define M_GN 450.0f
    #define M_GNZ 400.0f
#elif GNbits == 5
    #define M_FS 4.7f
    #define M_GN 400.0f
    #define M_GNZ 355.0f
#elif GNbits == 6
    #define M_FS 5.6f
    #define M_GN 330.0f
    #define M_GNZ 295.0f
#elif GNbits == 7
    #define M_FS 8.1f
    #define M_GN 230.0f
    #define M_GNZ 205.0f
#endif

//Gyro Cal
//set the Gyro full scale range (dps)
#define G_FS 500
//get senitivity  (mdps/LSB)
#if G_FS == 250
    #define So_G 8.75f
#elif G_FS == 500
    #define So_G 17.50f
#elif G_FS == 2000
    #define So_G 70.0f
#endif
    
///////////////////////////////////////////////////////////////////    
//Outputs

//Gyroscope output
#define OUT_X_L_G 0xa8
#define OUT_X_H_G 0xa9
#define OUT_Y_L_G 0xaa
#define OUT_Y_H_G 0xab
#define OUT_Z_L_G 0xac
#define OUT_Z_H_G 0xad

//Accelerometer output
//change 2 to a to alllow for block reads
#define OUT_X_L_A 0xa8
#define OUT_X_H_A 0xa9
#define OUT_Y_L_A 0xaa
#define OUT_Y_H_A 0xab
#define OUT_Z_L_A 0xac
#define OUT_Z_H_A 0xad

//Magnetometer output
#define OUT_X_H_M 0x03
#define OUT_X_L_M 0x04
#define OUT_Z_H_M 0x05
#define OUT_Z_L_M 0x06
#define OUT_Y_H_M 0x07
#define OUT_Y_L_M 0x08

///////////////////////////////////////////////////////////////////////////////////////
//timing functions
double get_time();

///////////////////////////////////////////////////////////////////////////////////////////////////
//read devices
void i2cOpen();
void i2cClose();
void i2cSetAddress(int address);
void writeRegister(uint8_t reg, uint8_t value);
void read6Registers(uint8_t reg,int mag);
int16_t readRegisterPair(uint8_t reg, int mag);
uint8_t readRegister(uint8_t reg);
float getMagSensitivity(int gain);
float getAccelSensitivity(int afs_sel);
float getGyroSensitivity(int fs_sel);
void printYPRtoFile(float * ypr);
void setAccel();
void setMag();
void setGyro();
void * readIMUData(void * arg);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//FIFO  conversion functions
void fifo_out(char *message);
void YPR_to_socket_format(float yaw, float pitch, float roll, char * message);
void raw_data_format(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, char * message);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//orientation files
float invSqrt(float number);
void getOrientation(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float q0, float q1, float q2, float q3);
void arr3_rad_to_deg(float * arr);
void printFloatArray(float *q, int size);
float * zeroAccel();
void zeroGyro();
void yawPitchRoll();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//control loop files

//getForce:
//"Calculates the force needed by taking the inner product of the k and v vectors.
// Outputs the force that needs to be applied to the ball"
//k1,k2,k3,k4 -> "elements of the k vector (linear position, linear velocity, angular position, angular velocity)"
//x1,x2,x3,x4 -> "elements of the x vector (linear position, linear velocity, angular position, angular velocity)"
float getForce(float k1,float k2,float k3,float k4,float x1,float x2, float x3,float x4);

//mapForceToVelocity:
//"numerically integrate the Force to get velocity and outputs the needed ball velocity"
//force -> "value calculated from getForce function"
//velocity -> "the linear velocity of the ball"
//CONTROL_FREQUENCY -> the frequency at which the control program fires
float mapForceToVelocity(float force, float velocity);

//mapVelocityToMotorVelocity
//    Inputs:        vel_x - rolling velocity in robot's forward direction, inches/sec
//                   vel_y - rolling velocity in robot's left direction, inches/sec
//                   omega_z - angular velocity about robot's Z axis, deg/sec
//    
//     Outputs:      motor_cps[1] - speed of forward motor, counts per second
//                   motor_cps[2] - speed of right motor, counts per second
//                   motor_cps[3] - speed of left motor, counts per second
//
//     Coordinates:  x - forward, y - left, z - up
//     Motors:       a - forward, b - right, c - left
void mapVelocityToMotorVelocity(float vel_x, float vel_y, float omega_z);
