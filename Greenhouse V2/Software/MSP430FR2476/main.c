#include <msp430.h> 
#include "greenhouse.h"

void test();

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	initialize(1);
	test();
	while(1);
	return 0;
}

void test(){

}
