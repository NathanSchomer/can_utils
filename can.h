/*
 * tick.h
 *
 *  Created on: Feb 22, 2012
 *      Author: Rich Primerano
 */

#ifndef CAN_H_
#define CAN_H_

#include "board.h"

// allow CAN buffer to be interpreted as different data types
typedef union  {
	unsigned char asChars[8];
	int asInts[2];
	long asLong[2];
} canBuff;

typedef struct {
	char dest;
	char source;
	char IID;
	char length;
	void* data;
} canBuffer;

canBuff txBuffer, rxBuffer;

long otherMotPos;

void canRxISR (void);
void canTx (char, char, char, void*, char);

void canCfg(char boardID)
{
	tCANMsgObject sCANMessage;

    // Configure CAN Pins
    GPIOPinTypeCAN(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // The GPIO port and pins have been set up for CAN.  The CAN peripheral
    // must be enabled.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

    // Reset the state of all the message object and the state of the CAN
    // module to a known state.
    CANInit(CAN0_BASE);

    // Configure the bit rate for the CAN device, the clock rate to the CAN
    // controller is fixed at 8MHz for this class of device and the bit rate is
    // set to 250000.
    CANBitRateSet(CAN0_BASE, 8000000, 1000000);

    // Enable interrupts for the CAN in the NVIC.
    CANIntRegister(CAN0_BASE, canRxISR);

    // Enable interrupts from CAN controller.
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR);
    IntEnable(INT_CAN0);

    // Take the CAN0 device out of INIT state.
    CANEnable(CAN0_BASE);


    // Two message objects will be set up. One receives all messages with ID

    sCANMessage.ulMsgIDMask 	= 0x007;          			// mask, lower 3 bits are destination address
    sCANMessage.ulFlags 		= MSG_OBJ_RX_INT_ENABLE | 	// receiver buffer with interrupt
    								MSG_OBJ_USE_ID_FILTER;	// message filtering
    sCANMessage.ulMsgLen 		= 8;                   		// allow up to 8 bytes

    // RX buffer - broadcast messages go to address 7 and land in buffer 1
    sCANMessage.ulMsgID 		= 0x007;
    CANMessageSet(CAN0_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_RX);

    // RX buffer - device specific messages go to dip switch address and land in buffer 2
    sCANMessage.ulMsgID 		= boardID;
    CANMessageSet(CAN0_BASE, 2, &sCANMessage, MSG_OBJ_TYPE_RX);

    //TX buffer - buffer 3
    //sCANMessage.ulFlags 		= MSG_OBJ_TX;
    //CANMessageSet(CAN0_BASE, 3, &sCANMessage, MSG_OBJ_TYPE_RX);
}

// determine what type of CAN message was received and call appropriate handler
void canRxISR(void) {

	unsigned long ulStatus;
	tCANMsgObject sCANMessage;
	canBuff canData;

	sCANMessage.pucMsgData = canData.asChars;			// link message object to buffer

	IntMasterDisable();

	ulStatus = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

	ledOn(1);

	// has a CAN error occured?
	if(ulStatus == CAN_INT_INTID_STATUS) {
		ulStatus = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
		ledOn(0);
	}

	// is this a broadcast message?
	else if(ulStatus == 1) {
		CANMessageGet(CAN0_BASE, 1, &sCANMessage, 0);		// transfer data to CAN struct

		otherMotPos = canData.asLong[0];					//store the remote motor's position in this global

		CANIntClear(CAN0_BASE, 1);
	}

	// is this message addressed to just this device?
	else if(ulStatus == 2) {
		CANMessageGet(CAN0_BASE, 2, &sCANMessage, 0);		// transfer data to CAN struct

		CANIntClear(CAN0_BASE, 2);
	}

	else {
//		ledOn(0);
	}

	IntMasterEnable();

}

void canTx (char dest, char src, char IID, void* data, char len) {

	tCANMsgObject sCANMessage;

	// MID = IID | src | dest;
	long MID = (IID & 0x1f) << 6;
	MID = MID | ((src & 0x07) << 3);
	MID = MID | (dest & 0x07);


	sCANMessage.ulMsgID 		= MID;            	// broadcast message address
	sCANMessage.ulMsgIDMask 	= 0x007;            // mask lower 3 bits
	sCANMessage.ulFlags = MSG_OBJ_USE_ID_FILTER;	//
	sCANMessage.ulMsgLen = len;       				// 4 bytes sent, 1 int
	sCANMessage.pucMsgData = data;      			// ptr to message content

	IntMasterDisable();
	CANMessageSet(CAN0_BASE, 3, &sCANMessage, MSG_OBJ_TYPE_TX);
	IntMasterEnable();
}

void canTxNew (canBuffer* buff) {

	tCANMsgObject sCANMessage;

	// Pack instruction ID, source, and destination into CAN message ID (MID) field
	long MID = (buff->IID & 0x1f) << 6 | ((buff->source & 0x07) << 3) | (buff->dest & 0x07);
	
	// Configure CAN data structure
	sCANMessage.ulMsgID 		= MID;            	// broadcast message address
	sCANMessage.ulMsgIDMask 	= 0x007;            // mask lower 3 bits
	sCANMessage.ulFlags = MSG_OBJ_USE_ID_FILTER;	// use message object mask
	sCANMessage.ulMsgLen = buff->length;       		// length of message
	sCANMessage.pucMsgData = buff->data;      		// ptr to message content

	// Ensure we have exclusive access to CAN and transmit data
	IntMasterDisable();
	CANMessageSet(CAN0_BASE, 3, &sCANMessage, MSG_OBJ_TYPE_TX);
	IntMasterEnable();
}

#endif /* CAN_H_ */
