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

typedef union
{
  char c[8];
  int i[2];
  float f[2];
  double d;
  unsigned long long ull;
  unsigned long int uli[2];
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


struct can_frame *setCanFrame(int command, int src, int dest, char * data, int length);
 
int main(int argc, char **argv)
{
	int command;
	int src;
	int dest;
    //define the union used to pass the payload
    canData data;
	int length;
		//read in the the cmd line args
        //6 arguments assumes the payload is a unsigned long long
        //7 arguments assumes the payload is an unsigned long int followed by a float
		if (argc != 6 && argc != 7){
			printf("Wrong number of arguments");
			return 1;
		}
        //6 arg sends an unsigned long long
		else if (argc ==6){
			command = atoi(argv[1]);
			src = atoi(argv[2]);
			dest = atoi(argv[3]);
			data.ull = strtoull(argv[4], NULL, 0);
			length = atoi(argv[5]);
		}
        //7 arg sends an unsigned long int followed by a float
        else{
            command = atoi(argv[1]);
			src = atoi(argv[2]);
			dest = atoi(argv[3]);
			data.uli[0] = strtoul(argv[4], NULL, 0);
            data.f[1] = atof(argv[5]);
			length = atoi(argv[6]);
        }
		//check that payload length is 8 or less
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
 
        printf("%s at index %d\n", ifname, ifr.ifr_ifindex);
 
        if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                perror("Error in socket bind");
                return -2;
        }
		
		frame = setCanFrame(command,src,dest, data.c,length);
		
        nbytes = write(s, frame, sizeof(struct can_frame));
 
        printf("Wrote %d bytes\n", nbytes);
		free(frame);
        return 0;
}
//length is given in bytes
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

