
#include "board.h"
#include "tick.h"
#include "leds.h"
#include "hall.h"
#include "interrupts.h"
#include "motor.h"
#include "quad.h"
#include "can.h"
#include "analog.h"
//#include "eth.h"
//#include "httpComm.h"

char boardID;
ISRflags flags;
long set = 0;
unsigned long adcData[8];
canBuffer txB;

static long counter;

// interrupt service routines
void sysTickISR(void);
void hallChangeISR(void);

void motorUpdate (void);


int main(void) {

    // If running on Rev A2 silicon, turn the LDO voltage up to 2.75V.
    if(REVISION_IS_A2) {SysCtlLDOSet(SYSCTL_LDO_2_75V);}

    // Configure the processor to run at 50 MHz.
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ);

    // prepare peripherals for initialization and set interrupt priorities
    peripherals();

    // configure all peripherals
    IOCfg();									// LEDs and button
    sysTickCfg();								// 100 Hz housekeeping clock
    hallCfg();									// hall sensors
    quadCfg();									// quadrature counters
    motorPWMCfg();								// PWM motor drivers
    analogCfg();								// analog inputs
    //ethernetCfg();								// ethernet

    //motorEnable();

    canCfg(getID());							// initialize CAN bus
	
	// Initalize the CAN TX buffer structure
	txB.dest = 7;
	txB.source = getID();
	txB.IID = 1;
	txB.data = &counter;
	txB.length = sizeof(counter);

	IntMasterEnable();							// enable interrupts
	
    while(1) {

    	// handle peridic tasks
		if(flags.tick) {

			if (pbRead()) {
				ledOn(0);
			}
			else {
				ledOff(0);
			}

			counter++;
			canTxNew(&txB);

			ledTog(1);

			flags.tick = 0;
		}

		// process new received CAN data
		if(flags.canRX) {

			flags.canRX = 0;
		}

		// go to sleep if no additional flags need processing
		if(!flags.all) {
			SysCtlSleep();
		}

    }

}

void motorUpdate (void) {
	float voltage;

	long motorPos, motorVel;

	long err, err_diff;
	static long err_old, err_int;

	#define PGAIN	0.001
	#define IGAIN	0
	#define DGAIN 	0

	// read motor position and velocity
	motorPos = QEIPositionGet(QEI0_BASE);
	motorVel = QEIVelocityGet(QEI0_BASE);

	//read bus voltage and current sensors
	analogGetData(adcData);
	analogTrig();

	//canTx (7, getID(), 0x55, &motorPos, 4);
	
	// Prepare and send CAN data
	txB.IID = 1;
	txB.data = &motorPos;
	txB.length = sizeof(motorPos);
	canTxNew(&txB);
	
	// update P, I, and D terms
	err = motorPos - otherMotPos;
	err_diff = err - err_old;
	err_int += err;
	err_old = err;

	voltage = err * PGAIN + err_int * IGAIN + err_diff * DGAIN;

	motorPowerSet(voltage);

}

void sysTickISR(void) {

	static int tickCount = TICK_PERIOD;

	if (!tickCount--) {

		tickCount = TICK_PERIOD
		flags.tick = 1;
	}

	//motorUpdate();

}

void hallChangeISR(void) {

	trapazodComm();
	GPIOPinIntClear(PIN_HALLA_PORT, (PIN_HALLA_PIN | PIN_HALLB_PIN | PIN_HALLC_PIN));
}
