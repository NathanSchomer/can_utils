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

//Print recieved CAN message
int printCANFrame(struct can_frame frame);
void toggleMotor(int motor, int on);
void setMotorVelocity(signed long int velocity);
void setP(int value);
void setI(int value);
void setD(int value);
 
int main(int argc, char **argv)
{
        int numberOfMessagesToRead;
        if (argc != 2){
            printf("Please enter 1 argument to set number of messages to read\n");
            return 1;
        }
        numberOfMessagesToRead = atoi(argv[1]);
        int s;
        int nbytes;
        struct sockaddr_can addr;
        struct can_frame frame;
        struct ifreq ifr;
 
        char *ifname = "can0";
 
        //opena socket using CAN_RAW protocal
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
        while(numberOfMessagesToRead > 0){
            nbytes = read(s, &frame, sizeof(struct can_frame));
            if (nbytes < sizeof(struct can_frame)) {
                printf("Incomplete CAN frame");
                return 1;
            }
            else{
                printCANFrame(frame);
                numberOfMessagesToRead--;
            }
        }
}

int printCANFrame(struct can_frame frame){
    int i;
    if (frame.can_dlc > 8) {
        printf("invalid can message length");
        return 1;
    }    
    printf("<0x%x> ",frame.can_id);
    printf("[%x] ",frame.can_dlc);
    for(i = 0; i< frame.can_dlc;i++){
        printf("%x ",frame.data[i]);
    }
    printf("\n");
}
