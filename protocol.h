/*
 * protocol.h
 *
 *  Created on: 01.12.2012
 *      Author: Appl
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

void protocol(unsigned char packet[], unsigned int size, unsigned int channel);
void send_port2_status(int port_stat, unsigned int channel);

#endif /* PROTOCOL_H_ */
