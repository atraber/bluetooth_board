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

void l2cap_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
int port2_poll(struct data_source *ds);

#endif /* PROTOCOL_H_ */
