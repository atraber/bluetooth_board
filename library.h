/*
 * library.h
 *
 *  Created on: 01.11.2012
 *      Author: Appl
 */

#ifndef LIBRARY_H_
#define LIBRARY_H_


//void configureClocks(); 		defined in hal_init
void initSwitch(int group, int pin);	//initialize switch
void initOutput(int group, int pin);	//initialize output
void setOutput(int group, int pin, int val);	//change value of output
void toggle(int group, int pin);	//toggle value of Output
int switchval(int group, int pin);		//return current switch value
void clearflag(int group, int pin);		//clears interrupt flags

#endif /* LIBRARY_H_ */
