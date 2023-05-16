#include <msp430.h> 
#include "greenhouse.h"

void test();

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	initialize(1);
	test();
    _bis_SR_register(LPM0_bits | GIE);
	return 0;
}

void test(){
    sensors[0].system_num = 'a';
    tst = sizeof(sensors);
    uartWriteSensors();

}
