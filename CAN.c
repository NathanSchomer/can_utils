    #include "canHeader.h"

    int main(int argc, char **argv)
    {

    }

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
     
            printf("%s at index %d\n", ifname, ifr.ifr_ifindex);
     
            if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                    perror("Error in socket bind");
                    return -2;
            }
            
            frame = setCanFrame(command, src, dest, data, length);
            
            nbytes = write(s, frame, sizeof(struct can_frame));
     
            printf("Wrote %d bytes\n", nbytes);
            free(frame);
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

    void echo(int motor){
        if (motor > 3 || motor < 0) {
            printf("Invalid motor ID.  Must be number from 1 to 3. \n");
            return -1;
        }
        
        canData data;
        //set canData to 0
        data.ull = 0;
        canSend(ECHO, BB_ID, motor, data.c, 4)
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
        canSend(TOGGLE_MOTOR, BB_ID, motor, data.c, 4);
    }

    void setMotorVelocity(int motor, signed long int velocity){
        canData data;
        data.li[0] = veocity;
        data.li[1] = 0;
        canSend(SET_VELOCTIY, BB_ID, motor, data.c, 4);
    }

    int setGain(int motor, int param, float value){
        if (motor > 3 || motor < 0) {
            printf("Invalid motor ID.  Must be number from 1 to 3. \n");
            return -1;
        }
        else if (param > 3 || param < 0) {
            printf("Invalid param ID.  Must be number from 1 to 3. \n");
            return -2;
        }
        else {
            canData data;
            canData.f[1] = value
            canData.i[0] = param;
            canSend(SET_GAIN, BB_ID, motor, data.c[0], 8);
            return 0;
        }
    }