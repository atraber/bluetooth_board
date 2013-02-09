/*
 * library.c
 *
  *  Created on: 01.11.2012
 *      Author: Appl
 */
#include "msp430f5438a.h"
#include "library.h"

/*
 * defined in hal_init
 void configureClocks()
 {
	P7SEL |= 0x03;          // Analog function for XT1 Pins
	UCSCTL6 &= ~(XT1OFF);   // XT1 On
	UCSCTL6 |= XCAP_3;   	// Set capacity

	UCSCTL4 |= SELA_0;		// set ACLK to XT1

	P11DIR |= BIT0;         // set pin 11.0 to output
  	P11SEL &= ~BIT0;		// ACLKset out to pin 11.0

 }*/

void initOutput(int group, int pin)
{
	if (group==1)
	{
		P1SEL &= ~pin; // set pin to gpio
		P1DIR |= pin; // Set pin to output direction
	}
	if (group==2)
	{
		P2SEL &= ~pin;// set pin to gpio
		P2DIR |= pin; // Set pin to output direction
	}
}

void setOutput(int group, int pin, int val)
{
	if (group==1)
	{
		if (val==0)
			P1OUT &= ~pin;
		if (val==1)
			P1OUT |= pin;
	}
	if (group==2)
	{
		if (val==0)
			P2OUT &= ~pin;
		if (val==1)
			P2OUT |= pin;
	}
}

void toggle(int group, int pin)
{
	if (group==1)
		P1OUT = P1OUT^pin;
	if (group==2)
		P2OUT = pin^P2OUT;
}

void initSwitch(int group, int pin)
{/*
	if (group==3)
	{
		P3SEL &= ~pin;	// set pin to gpio
		P3REN |= pin;
		P3OUT |= pin;	//set pullup
	}*/
	if (group==2)
	{

		P2SEL &= ~pin;	// set pin to gpio
		P2REN |= pin;
		P2OUT |= pin;	//set Pullup
		//P2IE  |= pin;	//interrupt enabled
		//P2IES |= pin;	//Hi/Lo falling edge
		//P2IFG & ~pin;	//IFG cleared just in case
	}
}

int switch_value(int group, int pin)
{
	if (group==3)
		return !(P1IN & pin);
	if (group==2)
		return !(P2IN & pin);
	return 0;
}

void clearflag(int group, int pin)
{
	if (group==3)
		P1IFG &= ~pin;
	if (group==2)
		P2IFG &= ~pin;
}


