#include "i2cNew.h"
#include "canHeader.h"
#include<semaphore.h>
#include<pthread.h>

//semaphores
sem_t sem;

//threads
pthread_t thread0;

float motor_cps[3];
float k[4];
float q[4];
float ypr[3];
float mag_off[3];
float accel_off[3];
char FIFO_message[140];
int gyro_off[3];
int16_t values[3];
float valueXG;
float valueYG;
float valueZG;
float valueXA;
float valueYA;
float valueZA;
float valueXM;
float valueYM;
float valueZM;
float xPos;
float yPos;
float xVelo;
float yVelo;
float omegaZ;

//////////////////////////////////////////////////////////////////////////////////////////
int printCANFrame(struct can_frame frame){
	int i;
	if (frame.can_dlc > 8) {
		printf("invalid can message length\n");
		return 1;
	}
	printf("<0x%x> ",frame.can_id);
	printf("[%x] ",frame.can_dlc);
	for(i = 0; i< frame.can_dlc;i++){
		printf("%x ",frame.data[i]);
	}
	printf("\n");
}

struct can_frame *setCanFrame(int command, int src, int dest, char * data, int length){
	int i;
	struct can_frame *frame = (struct can_frame *)malloc(sizeof(struct can_frame));
	
	frame->can_id  =(command & 0x1f) << 6 | ((src & 0x07) << 3) | (dest & 0x07);
	frame->can_dlc = length;
	for(i = 0; i < length;i++){
		frame->data[i] = data[i];
	}
	return frame;
}

int canSend(int command, int src, int dest, char * data, int length){
		//check that length is 8 or less
		if (length > 8){
			printf("message length is too long (8 bytes or less)\n");
			return 1;
		}
		int s;
		int nbytes;
		struct sockaddr_can addr;
		struct can_frame * frame;
		struct ifreq ifr;
 
		char *ifname = "can0";
 
		if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
				perror("Error while opening socket");
				return -1;
		}
 
		strcpy(ifr.ifr_name, ifname);
		ioctl(s, SIOCGIFINDEX, &ifr);
 
		addr.can_family  = AF_CAN;
		addr.can_ifindex = ifr.ifr_ifindex; 
 
	//	printf("%s at index %d\n", ifname, ifr.ifr_ifindex);
 
		if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
				perror("Error in socket bind");
				return -2;
		}
		
		frame = setCanFrame(command, src, dest, data, length);
		
		nbytes = write(s, frame, sizeof(struct can_frame));
 
	//	printf("Wrote %d bytes\n", nbytes);
		free(frame);
		close(s);
		return 0;
}

int canRecieve(int numberOfMessages){
	int numberOfMessagesToRead;
		if (numberOfMessages < 1){
			printf("number of messages must be greater than 0\n");
			return 1;
		}
		
		int s;
		int nbytes;
		struct sockaddr_can addr;
		struct can_frame frame;
		struct ifreq ifr;
 
		char *ifname = "can0";
 
		//open a socket using CAN_RAW protocal
		if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
				perror("Error while opening socket");
				return -1;
		}
		
		strcpy(ifr.ifr_name, ifname);
		ioctl(s, SIOCGIFINDEX, &ifr);
		
		//set the CAN protocal family to use
		addr.can_family  = AF_CAN;
		addr.can_ifindex = ifr.ifr_ifindex; 
		
		printf("%s at index %d\n", ifname, ifr.ifr_ifindex);
		
		//attmept to bind to the socket and establish communication
		//with the CAN interface
		if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
				perror("Error in socket bind");
				return -2;
		}
		//read a CAN frame from the CAN bus
		//check that bytes recieved is equal to the size of a can frame
		while(numberOfMessages > 0){
			nbytes = read(s, &frame, sizeof(struct can_frame));
			if (nbytes < sizeof(struct can_frame)) {
				printf("Incomplete CAN frame");
				return 1;
			}
			else{
				printCANFrame(frame);
				numberOfMessages--;
			}
		}
}

int echo(int motor){
	if (motor > 3 || motor < 0) {
		printf("Invalid motor ID.  Must be number from 1 to 3. \n");
		return -1;
	}
	
	canData data;
	//set canData to 0
	data.ull = 0;
	canSend(ECHO, BB_ID, motor, data.c, 8);
	return 0;
}

void setMode(int motor, int mode){
    canData data;
	//set canData to 0
	data.ull = mode;
    canSend(MODE,BB_ID, motor, data.c, 8);

}

void setLED(int motor, int on){
    canData data;
	data.ull = on;
	canSend(LED, BB_ID, motor, data.c, 8);
}

void toggleMotor(int motor, int on){
	if(on == 1){
		printf("toggling motor on\n");
	}
	else{
		printf("toggling motor off\n");
	}
	canData data;
	//set canData to 0
	data.ull = on;
	canSend(TOGGLE_MOTOR, BB_ID, motor, data.c, 8);
}

void setMotorVelocity(int motor, signed long int velocity){
	canData data;
	data.li[0] = velocity;
	data.li[1] = 0;
	canSend(SET_VELOCITY, BB_ID, motor, data.c, 8);
}

void setGain(int motor, int param, float value){
	if (motor > 3 || motor < 0) {
		printf("Invalid motor ID.  Must be number from 1 to 3. \n");
	}
	else if (param > 3 || param < 0) {
		printf("Invalid param ID.  Must be number from 1 to 3. \n");
	}
	else {
		canData data;
		data.f[1] = value;
		data.i[0] = param;
		canSend(SET_GAIN, BB_ID, motor, data.c, 8);
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
void getOrientation(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float q0, float q1, float q2, float q3) {
  float i;
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
  float qa, qb, qc;

  float integralFBx, integralFBy, integralFBz;
  float sampleFreq = 200; //figure out if in Hz
  
  //current best setting
  float twoKp = (2.0f *0.5f);
  float twoKi = 0;//(2.0f *0.01f);
  
  
  gx *= 3.14159265f/180;
  gy *= 3.14159265f/180;
  gz *= 3.14159265f/180;
  
  integralFBx = 0.0f;
  integralFBy = 0.0f;
  integralFBz = 0.0f;
  
  q0q0 = q0 * q0;
  q0q1 = q0 * q1;
  q0q2 = q0 * q2;
  q0q3 = q0 * q3;
  q1q1 = q1 * q1;
  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q2q2 = q2 * q2;
  q2q3 = q2 * q3;
  q3q3 = q3 * q3;
  
 
  
  // read from the mag if valid reading
  if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)) {
    float hx, hy, bx, bz;
    float halfwx, halfwy, halfwz;
    
    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;
    
    // Reference direction of Earth's magnetic field
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
	bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
    
	
    // Estimated direction of magnetic field
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
    
		
	
    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex = (my * halfwz - mz * halfwy);
    halfey = (mz * halfwx - mx * halfwz);
    halfez = (mx * halfwy - my * halfwx);
	
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f)) {
    float halfvx, halfvy, halfvz;
    
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    
    // Estimated direction of gravity
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;

    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex += ((ay * halfvz) - (az * halfvy));
    halfey += ((az * halfvx) - (ax * halfvz));
    halfez += ((ax * halfvy) - (ay * halfvx));
  }
 

  if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * (halfex * (1.0f / sampleFreq));	  // integral error scaled by Ki
      integralFBy += twoKi * (halfey * (1.0f / sampleFreq));
      integralFBz += twoKi * (halfez * (1.0f / sampleFreq));

      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;

    }
    else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
  

  
  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);
  

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

  //calculates Roll-Pitch-Yaw
void yawPitchRoll() {
  float gx, gy, gz; // estimated gravity direction
  
  
  gx = 2 * (q[1]*q[3] - q[0]*q[2]);
  gy = 2 * (q[0]*q[1] + q[2]*q[3]);
  gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  
  ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1)*180/PI;
  ypr[1] = atan2(gx,sqrt(gy*gy + gz*gz))*180/PI;
  ypr[2] = atan2(gy,sqrt(gx*gx + gz*gz))*180/PI;
}


float invSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;

  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  return y;
}


void arr3_rad_to_deg(float * arr) {
  arr[0] *= 180/3.14159265f;
  arr[1] *= 180/3.14159265f;
  arr[2] *= 180/3.14159265f;
}

void  printFloatArray(float *q, int size){
  int i = 0;
  for(i = 0; i < size; i++){
    //printf("%f\n",q[i]);
  }
	//printf("\n");
}

//control loop functions

//getForce:
//"Calculates the force needed by taking the inner product of the k and v vectors.
// Outputs the force that needs to be applied to the ball"
//k1,k2,k3,k4 -> "elements of the k vector (linear position, linear velocity, angular position, angular velocity)"
//x1,x2,x3,x4 -> "elements of the x vector (linear position, linear velocity, angular position, angular velocity)"
float getForce(float k1,float k2,float k3,float k4,float x1,float x2, float x3,float x4){
    float force;
    force = k1*x1+k2*x2+k3*x3+k4*x4;
    return force;
}

//mapForceToVelocity:
//"numerically integrate the Force to get velocity and outputs the needed ball velocity"
//force -> "value calculated from getForce function"
//velocity -> "the linear velocity of the ball"
float mapForceToVelocity(float force, float velocity){
 
    float veloFinal = (velocity+(ACCEL_GAIN)*force*(1/CONTROL_FREQUENCY));
    if (veloFinal > MAX_VELOCITY){
        return MAX_VELOCITY;
    }
    else if(veloFinal < -MAX_VELOCITY){
		return -MAX_VELOCITY;
    }
    else{
        return veloFinal;
    }
}


void mapVelocityToMotorVelocity(float vel_x, float vel_y, float omega_z){
	int i;
	float gear_ratio = (CLUSTER_GEAR / PINION) * (OUTPUT_PULLEY/CLUSTER_PULLEY) * (D_BALL/D_WHEEL); 
	float omega_x;
	float omega_y;
	
    // Translate ball's X and Y velocities into X and Y angualr rates of
    omega_z =  (omega_z*PI)/180;
	omega_x = -2*vel_y/D_BALL;
	omega_y =  2*vel_x/D_BALL;
	
	
	// T(3x3)*omega_vector
	omega_x =  0.9428*omega_x + -0.4714*omega_z;
	omega_y = -0.4714*omega_x + 0.8165*omega_y - 0.4714*omega_z;
	omega_z = -0.4714*omega_x - 0.8165*omega_y - 0.4714*omega_z;
	
	//convert motor velocities to counts per second
	motor_cps[0] = omega_x/(2*PI)*gear_ratio*ENCODER_CPR;
    motor_cps[1] = omega_y/(2*PI)*gear_ratio*ENCODER_CPR;
    motor_cps[2] = omega_z/(2*PI)*gear_ratio*ENCODER_CPR;
    //limit check
     for(i =0; i < 3;i++){
	if (motor_cps[i] > MAX_CPS){
		motor_cps[i] = MAX_CPS;
	}
	else if (motor_cps[i] < -MAX_CPS){
		motor_cps[i] = -MAX_CPS;
	}
	else{
	}
     }
}

// I2C Linux device handle
int i2cFile;

// open the Linux device
void i2cOpen() {
	i2cFile = open("/dev/i2c-1", O_RDWR);
	if (i2cFile < 0) {
		perror("i2cOpen");
		exit(1);
	}
}

// close the Linux device
void i2cClose() {
	close(i2cFile);
}

//set the address of the device
void i2cSetAddress(int address){
	if (ioctl(i2cFile, I2C_SLAVE, address) < 0) {
		perror("i2cSetAddress error");
		exit(1);
	}
}

//write to a single register
void writeRegister(uint8_t reg, uint8_t value){
	uint8_t data[2];
	data[0] = reg;
	data[1] = value & 0xff;
	if (write(i2cFile, data, 2) != 2) {
		perror("setRegister error");
	}
}

// Read two bytes in a row (MSB followed by LSB)
int16_t readRegisterPair(uint8_t reg,int mag){
	//might need to be an array of three?
	uint8_t data[2];
	data[0] = reg;
	//1 is the number of bytes written
	if (write(i2cFile, data, 1) != 1) {
		perror("set register error");
	}
    
	if (read(i2cFile, data, 2) != 2) {
		perror("value read error");
	}
	//converts the two bytes (MSB followed by LSB)
    if (mag ==1){
        //printf("Mag: %x ",data[0]);
        //printf("%x\n",data[1]);
        return data[1] | (data[0] << 8);
    }
    //converts the two bytes (LSB followed by MSB)
    else{
       // printf("Accel: %x ",data[0]);
       // printf("%x\n",data[1]);
        return data[0] | (data[1] << 8);
    }
}

void read6Registers(uint8_t reg,int mag){
	//read all six registers for each sensor
	int i;
    uint8_t data[6];
	data[0] = reg;
	if(write(i2cFile,data,1) != 1){
		perror("set reg error");
	}
	if (read(i2cFile, data, 6) != 6){
		perror("value read error");
	}
	if (mag ==1){
		for(i=0;i<3;i++){
			values[i] = (data[2*i+1] | (data[2*i] << 8));
		}
	}
	else{
		for(i=0;i<3;i++){
			values[i] = (data[2*i] | (data[2*i+1] << 8));
		}
	}
}

uint8_t readRegister(uint8_t reg){
	//might need to be an array of three?
	uint8_t data[1];
	data[0] = reg;
	//1 is the number of bytes written
	if (write(i2cFile, data, 1) != 1) {
		perror("set register error");
	}
	if (read(i2cFile, data, 1) != 1) {
		perror("value read error");
	}
	//read a single byte
	return data[0];
}


void zeroGyro() {
  const int totSamples = 50;
  int16_t valueXG;
  int16_t valueYG;
  int16_t valueZG;
  int16_t tmpOffsets[3] = {0,0,0};
  int i;
  
   gyro_off[0] = 0;
   gyro_off[1] = 0;
   gyro_off[2] = 0;
  
  //set the gyro address  
  i2cSetAddress(GYRO_ADDR);
  
  for (i = 0; i < totSamples; i++){
    
		
    //read and print the Gyro values
	valueXG = readRegisterPair(OUT_X_L_G,0);
	valueYG = readRegisterPair(OUT_Y_L_G,0);
	valueZG = readRegisterPair(OUT_Z_L_G,0);
	/*
    printf("valueXG: %d ", valueXG);
    printf("valueYG: %d ", valueYG);
    printf("valueZG: %d \n", valueZG);
    */
    tmpOffsets[0] += valueXG;
    tmpOffsets[1] += valueYG;
    tmpOffsets[2] += valueZG;
  }
  
  gyro_off[0] = tmpOffsets[0] / totSamples;
  gyro_off[1] = tmpOffsets[1] / totSamples;
  gyro_off[2] = tmpOffsets[2] / totSamples;
  /*
    printf("valueXG: %d ", gyro_off[0]);
    printf("valueYG: %d ", gyro_off[1]);
    printf("valueZG: %d \n", gyro_off[2]);
    */
}

void fifo_out(char *message){
   FILE *fp; 
   char *fifoOut = message;
   
   if ((fp = fopen(FIFO_FILE, "w")) == NULL) {
	  perror("fopen");
	  exit(1);
   }

   fputs(fifoOut, fp);
  // printf("FIFO CLIENT ON '%s' SENT: %s\n",FIFO_FILE, fifoOut);
   fclose(fp);
}

void printYPRtoFile(float * ypr){
    FILE *fp;
    
    //use to only have one entry in the file
    fp = fopen("../testApp/public/imu_data.txt","w+");
    
    //use to append successful entries to the file
    //fp = fopen("imu_data.txt","a+");
    fprintf(fp, "%f %f %f\n",ypr[0],ypr[1],ypr[2]);
    fclose(fp);
}

void raw_data_format(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, char * message){
   
    char gx_string[20];
    char gy_string[20];
    char gz_string[20];
    char ax_string[20];
    char ay_string[20];
    char az_string[20];
    char mx_string[20];
    char my_string[20];
    char mz_string[20];
    
	char * data_break = " ";
    char * message_terminator = "P";	

    sprintf(gx_string,"%f",gx);
    sprintf(gy_string,"%f",gy);
    sprintf(gz_string,"%f",gz);
    sprintf(ax_string,"%f",ax);
    sprintf(ay_string,"%f",ay);
    sprintf(az_string,"%f",az);
    sprintf(mx_string,"%f",mx);
    sprintf(my_string,"%f",my);
    sprintf(mz_string,"%f",mz);

	strcpy(message, gx_string);
	strcat(message, data_break);
	strcat(message, gy_string);
	strcat(message, data_break);
	strcat(message, gz_string);
	strcat(message, data_break);
	strcat(message, ax_string);
	strcat(message, data_break);
	strcat(message, ay_string);
	strcat(message, data_break);
	strcat(message, az_string);
	strcat(message, data_break);
	strcat(message, mx_string);
	strcat(message, data_break);
	strcat(message, my_string);
	strcat(message, data_break);
	strcat(message, mz_string);
	strcat(message, message_terminator);

}

void YPR_to_socket_format(float yaw, float pitch, float roll, char * message){
   
    char yaw_string[50];
    char pitch_string[50];
    char roll_string[50];
    char * html_line_break = " ";
    char * message_terminator = "P";	

    sprintf(yaw_string,"%f",yaw);
    sprintf(pitch_string,"%f",pitch);
    sprintf(roll_string,"%f",roll);

	strcpy(message, yaw_string);
	strcat(message, html_line_break);
	strcat(message, pitch_string);
	strcat(message, html_line_break);
	strcat(message, roll_string);
	strcat(message, message_terminator);
}

void setAccel(){

    //Read the Accel
		i2cSetAddress(ACCEL_ADDR);
		
		//Configuration
		//output data rate/ enable axis
		writeRegister(0x20,0x97);
        //all HP Filtering disabled
		writeRegister(0x21,0x00);
        writeRegister(0x22,0x00);
        //Full scale range set to +/- 2 g
        //block update to 1
        //high res mode set to 1
        writeRegister(0x23,0x88);
        writeRegister(0x24,0x00);
        writeRegister(0x25,0x00);

}

void setMag(){

    //set to a single read mode?

    //Read the Mag
		i2cSetAddress(MAG_ADDR);
		
		//Configuration
		//disable temp sensor and set output rate to 75Hz
		writeRegister(0x00,0x18);
		//set gain field range to +/- 4 guass
		writeRegister(0x01,0x80);
        //disable sleep mode and enter continuous mode
        writeRegister(0x02,0x00);

}
void setGyro(){
    //Read the Gyro
        i2cSetAddress(GYRO_ADDR);
        
        //Configuration
        //760Hz/ 3s cutoff
        writeRegister(0x20,0xdf);
        //Disables all HP Filtering
        writeRegister(0x21,0x00);
        writeRegister(0x22,0x00);
        //continuous update data/fullscale set to 500 dps
        writeRegister(0x23,0x10);
        writeRegister(0x24,0x00);

}

double get_time(){
       struct timeval t;
	   struct timezone tzp;
	   gettimeofday(&t, &tzp);
       return t.tv_sec + t.tv_usec*1e-6;
}

void * readIMUData(void * arg){
    int16_t rawValueXG;
    int16_t rawValueYG;
    int16_t rawValueZG;
    int16_t rawValueXA;
    int16_t rawValueYA;
    int16_t rawValueZA;
    int16_t rawValueXM;
    int16_t rawValueYM;
    int16_t rawValueZM;


    //Read the Mag
    i2cSetAddress(MAG_ADDR);
    
    read6Registers(OUT_X_H_M,1);
    //Mag registers go High then Low
    //Read and print values
    rawValueXM = values[0];
    rawValueZM = values[1];
    rawValueYM = values[2];
    sem_wait(&sem);
    valueXM = rawValueXM/M_GN;
    valueZM = rawValueZM/M_GN;
    valueYM = rawValueYM/M_GNZ;
    sem_post(&sem);
    //Read the Accel
    i2cSetAddress(ACCEL_ADDR);
    
    //set sample rate to 
    read6Registers(OUT_X_L_A,0);	
    //read and print values
    rawValueXA = values[0];
    rawValueYA = values[1];
    rawValueZA = values[2];
    //assuming 12-bit percession so the raw value is bit shifted 4 places
    sem_wait(&sem);
    valueXA = (rawValueXA>>4)*So_A/1000.0f;
    valueYA = (rawValueYA>>4)*So_A/1000.0f;
    valueZA = (rawValueZA>>4)*So_A/1000.0f;
    sem_post(&sem);
  
    //Read the Gyro
    i2cSetAddress(GYRO_ADDR);
    
    read6Registers(OUT_X_L_G,0);
    //read and print the Gyro values
    rawValueXG = values[0] - gyro_off[0];
    rawValueYG = values[1] - gyro_off[1];
    rawValueZG = values[2] - gyro_off[2];
    
    sem_wait(&sem);
    valueXG = rawValueXG*So_G/1000.0f;
    valueYG = rawValueYG*So_G/1000.0f;
    valueZG = rawValueZG*So_G/1000.0f;
    sem_post(&sem);
    
    return NULL;
}

int main(int argc, char** argv) {
    //timer stuff
    double start;
    double stop;
    double elapsed = 0;
    float sum = 0;
    
    //intializes the semaphore to be between threads and to set it's value to 1
    sem_init(&sem, 0, 1);
	int i;
    //open i2c for reading
	i2cOpen();
	q[0] = 1;
	q[1] = 0;
	q[2] = 0;
	q[3] = 0;
    
    //intialize k vector
    k[0] = 0;
    k[1] = 0;
    k[2] = 0.5;
    k[3] = 0;
    //intialize pos
    xPos = 0;
    yPos = 0;
	
    //Configure the devices
		setMag();
		setAccel();
        setGyro();
        zeroGyro();
    i = 0;
    //intial read so getOrientation has values to use
    readIMUData(NULL);
    	//7 is the braodcast address (toggles all motors)
	toggleMotor(7,1);
	while(i < 100000000){
        start = get_time();
        //create a thread to read IMU data
        pthread_create(&thread0, NULL, readIMUData, NULL);
       
	   //raw_data_format(valueXG, valueYG, valueZG, valueXA, valueYA, valueZA, valueXM, valueYM, valueZM, FIFO_message);
       
        //change so that the values are protecting during only the intial call of the Orientation function not it's entire execution
        sem_wait(&sem);
	    getOrientation(valueXG,valueYG,valueZG,valueXA,valueYA, valueZA, valueXM, valueYM, valueZM,q[0],q[1],q[2],q[3]);
        sem_post(&sem);
        yawPitchRoll(&q[0]);
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//START OF CONTROL CODE
		
        //update position and omega Z
        xPos = xPos + xVelo*(TIME_STEP);
        yPos = yPos + yVelo*(TIME_STEP);
        omegaZ = 0;
        
        //Diuble check accel
        //calc force needed for both x and y and then map the force to a velocity
        //ypr[1] = pitch  for x
        //ypr[2] = roll  for y
        xVelo = mapForceToVelocity(getForce(k[0], k[1], k[2], k[3],xPos, xVelo, ypr[1], valueXG), xVelo);
        yVelo = mapForceToVelocity(getForce(k[0], k[1], k[2], k[3],yPos, yVelo, ypr[2], valueYG), yVelo);
        //maps linear velo to motor velo (units?) stores result in motorVelo array in form [alphaVelocity, betaVelocity, gammaVelocity]
        mapVelocityToMotorVelocity((ypr[1]*abs(ypr[1]))/50.0f, (ypr[2]*abs(ypr[2]))/50.0f, omegaZ);
        
		
		//END OF CONTROL CODE
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //send the motor velocities to the motor driver boards
        //create function to convert rev/min to ticks/sec
        setMotorVelocity(1, motor_cps[0]);
        setMotorVelocity(2, motor_cps[1]);
        setMotorVelocity(3, motor_cps[2]);
		if((i%30) == 0){
			system("clear");
			printf("Mag \nAccel \nGyro \n");
			printf("%5.2f %5.2f %5.2f\n %5.2f %5.2f %5.2f\n %5.2f %5.2f %5.2f\n", valueXM, valueYM, valueZM, valueXA, valueYA, valueZA, valueXG, valueYG, valueZG);
			printf("yaw: %f\n", ypr[0]);
			printf("pitch %f\n", ypr[1]);
			printf("roll %f\n", ypr[2]);
			printf("x-Velo: %f\n",xVelo);
			printf("y-Velo: %f\n", yVelo);
			printf("x-ticks: %f\n",motor_cps[0]);
			printf("y-ticks: %f\n", motor_cps[1]);
			printf("z-ticks: %f\n", motor_cps[2]);
        }
			//printFloatArray(&q[0],4);
			//printFloatArray(yawPitchRoll(&q[0]),3);
            //printYPRtoFile(yawPitchRoll(&q[0]));
	        //******WRITE TO FIFO**********
		    YPR_to_socket_format(ypr[0],ypr[1],ypr[2],FIFO_message);
		    
            //fifo_out(FIFO_message);
        
        //wait for the IMU thread to finish and join them threads back together
        pthread_join(thread0, NULL);
		i++;
        stop = get_time();
        if((stop - start)>elapsed){
           elapsed = stop - start;
        }
        sum += elapsed;        
	}
    printf("max:%f\n",elapsed);
    printf("average: %f\n", sum/10000.0f);
	// close I2C device
	i2cClose();
	return 0;
}
//timer stuff
/*
loop{
double start;
    double stop;
    double elapsed = 0;
    float sum = 0;
    place at end:
    stop = get_time();
            if((stop - start)>elapsed){
               elapsed = stop - start;
            }
            sum += elapsed;
}           
printf("max:%f\n",elapsed);
printf("average: %f\n", sum/1000.0f);
*/
