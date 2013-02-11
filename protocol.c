
#include <msp430x54x.h>
#include <stdio.h>

#include "i2c_lib.h"
#include "l2cap.h"

//packet header variables
int type;
int len;
int seq;
int ownseq = 0;
unsigned int chan;

// variables used temporary but initialized only once
unsigned char packet[L2CAP_MINIMAL_MTU];


int l2cap_send_queued(uint16_t local_cid, uint8_t *data, uint16_t len);


void send_bt_request(unsigned char payload[], int paylen)
{
	//generate full package
	packet[0] = (type<<5) | (paylen+3);
	packet[1]=  ownseq;
	packet[2] = 0xff;
	memcpy(&packet[3], payload, paylen);
	//send packet

	// actually send the packet
	l2cap_send_queued(chan, packet, paylen+3);

	//update ownseq
	ownseq++;
	if (ownseq==0xff) //if 0xff reset
		ownseq=0;
}

void send_bt_response(unsigned char payload[], int paylen)
{
	//generate full package
	packet[0] = (type<<5) | (paylen+3);
	packet[1]=  ownseq;
	packet[2] = seq;
	memcpy(&packet[3], payload, paylen);
	//send packet

	// actually send the packet
	l2cap_send_queued(chan, packet, paylen+3);
	//update ownseq
	ownseq++;
	if (ownseq==0xff) //if 0xff reset
		ownseq=0;
}

void gpio_err(unsigned char payload[])	//error occured: not consistent
{
	payload[0] = payload [0] | 64; //ser error bit
	send_bt_response(payload, 2);
}


void get_header(unsigned char packet[])
{
	type = packet[0] >> 5;
	len = packet[0] & 0x1f;
	seq = packet[1];
	/*if(packet[2] != 0xff)	//received seq ack should be 0xff always
		header_err(packet);*/
}

void i2c_write(unsigned char addr, unsigned char command[], int size)		//to correct
{
	unsigned char package[256];
	int txlen = size-1;

	if(sendi2c(addr, &command[1], txlen))	//if return true, an error occured
		package[1] = 0x40;
	else
		package[1] = 0;

	//generate rest of answer
	package[0] = addr;
	memcpy(&package[2], &command[1], txlen); 		// int i; for (i=0;i<txlen;i++) package[i+2] = command[1+i];
	//send answer
	send_bt_response(package, txlen+2);
}

void i2c_read(unsigned char addr, unsigned char payload[], int size)
{
	unsigned char package[32];
	unsigned char rxdata[16];
	unsigned char rxlen = payload[0] & 31;
	int txlen = size-1;
	if(!sendi2c(addr, &payload[1], txlen))	//if sendi2c does not fail go on with get
	{
		if(geti2c(addr, rxdata, rxlen))		//set error bit if geti2c fails
			package[1] = rxlen | 0x40;
		else
			package[1] = rxlen;
	}
	else
		package[1] = rxlen | 0x40;		//sendi2c failed
	//generate answer
	size = 2 + rxlen + txlen;
	//unsigned char package[] = {((000 << 5) + len), packet[1], rxlen, txdata, rxdata} //how to put in data arrays???
	package[0] =  (addr | 128);
	memcpy(&package[2], &payload[1],txlen);		//int i; for (i=0;i<txlen;i++) package[i+2] = payload[1+i];
	memcpy(&package[2+txlen], rxdata, rxlen);		//for (i=0;i<rxlen;i++) package[i+2+txlen] = rxdata[i];
	//send answer
	send_bt_response(package, size);
}

void i2c_packet(unsigned char payload[], int size)
{
	int rw = payload[0] >> 7;
	unsigned char addr = payload[0] & 0x7f;
	if (rw)
		i2c_read(addr, &payload[1], size-1);
	else
		i2c_write(addr, &payload[1], size-1);
}

void gpio_send(int resp, int port_stat)
{
	if (resp)
	{
		unsigned char payload[2];
		payload[0] = (resp << 7) | 2;
		payload[1] = ~port_stat; //value has to be inverted as we detect low
		send_bt_response(payload, 2);
	}
	else
	{
		unsigned char payload[2];
		payload[0] = (resp << 7) | 2;
		payload[1] = ~port_stat; //value has to be inverted as we detect low
		send_bt_request(payload, 2);
	}
}

void gpio_request(unsigned char payload[])
{
	int pingroup = payload[0] & 31;
	if((payload[0] & 128) == 0 || pingroup != 2 ) //throw an error if not a request or pingroup not 2 as only one supported
		gpio_err(payload);
	gpio_send(1, P2IN);
}

void protocol(unsigned char packet[], unsigned int size, unsigned int channel)
{
	get_header(packet);
	chan = channel;
	switch (type)
	{
	case 0:	//i2c
		i2c_packet(&packet[3], size-3);
		break;

	case 1:	//gpio
		gpio_request(&packet[3]);
		break;

	default:
		break;
	}
}

void send_port2_status(int port_stat, unsigned int channel)
{
	chan = channel;
	type = 1;
	gpio_send(0, port_stat);
}



//value of port2 input
unsigned int port2_status;

int port2_poll(struct data_source *ds)
{
	if((P2IN & 0x0F) != port2_status)
	{
		// only check first 4 bits, ignore rest
		port2_status = P2IN & 0x0F;
		printf("status sent\n");
		send_port2_status(port2_status, chan);
	}

	return 0;
}



struct l2cap_packet
{
	uint16_t local_cid;
	uint8_t data[L2CAP_MINIMAL_MTU];
	uint16_t len;
};

#define QUEUE_SIZE 10

struct l2cap_packet l2cap_packet_queue[QUEUE_SIZE];
unsigned char l2cap_queue_index_insert;
unsigned char l2cap_queue_index_remove;

inline int l2cap_queue_full()
{
	return (QUEUE_SIZE + l2cap_queue_index_remove - l2cap_queue_index_insert) % QUEUE_SIZE == 1;
}

void l2cap_try_send_queued()
{
	if(l2cap_queue_index_insert == l2cap_queue_index_remove)
		return; // nothing to do, queue is empty

	if(l2cap_can_send_packet_now(l2cap_packet_queue[l2cap_queue_index_remove].local_cid))
	{
		l2cap_send_internal(l2cap_packet_queue[l2cap_queue_index_remove].local_cid, l2cap_packet_queue[l2cap_queue_index_remove].data, l2cap_packet_queue[l2cap_queue_index_remove].len);

		l2cap_queue_index_remove = (l2cap_queue_index_remove + 1) % QUEUE_SIZE;
	}
}

// returns 1 if packet can be sent, 0 if queue is full
int l2cap_send_queued(uint16_t local_cid, uint8_t *data, uint16_t len)
{
	l2cap_try_send_queued();

	if(l2cap_queue_full())
	{
		printf("Queue is full, cannot send\n");
		return 0; // queue is full
	}

	if(l2cap_queue_index_insert == l2cap_queue_index_remove && l2cap_can_send_packet_now(local_cid))
	{
		// packet can be sent directly as queue is empty
		printf("Packet sent directly\n");
		l2cap_send_internal(local_cid, data, len);

		return 1;
	}
	else
	{
		// insert packet into queue
		printf("Insert packet into queue\n");

		l2cap_packet_queue[l2cap_queue_index_insert].local_cid = local_cid;
		memcpy(l2cap_packet_queue[l2cap_queue_index_insert].data, data, len);
		l2cap_packet_queue[l2cap_queue_index_insert].len = len;

		l2cap_queue_index_insert = (l2cap_queue_index_insert + 1) % QUEUE_SIZE;

		return 1;
	}
}


extern bd_addr_t event_addr;

// Bluetooth logic
void l2cap_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    if (packet_type == HCI_EVENT_PACKET)
    {
    	switch(packet[0])
    	{
		case L2CAP_EVENT_INCOMING_CONNECTION:
			// data: event(8), len(8), address(48), handle (16),  psm (16), source cid(16) dest cid(16)
			bt_flip_addr(event_addr, &packet[2]);
			printf("L2CAP_EVENT_INCOMING_CONNECTION from %s, \n", bd_addr_to_str(event_addr));
			// accept
			l2cap_accept_connection_internal(channel);
			break;

		case L2CAP_EVENT_CHANNEL_OPENED:
			// inform about new l2cap connection
			if (packet[2] == 0)
			{
				printf("Channel successfully opened to %s\n", bd_addr_to_str(event_addr));

				// initialize values for new channel, old channel (if any) will no longer be used
				chan = channel;

				l2cap_queue_index_insert = 0;
				l2cap_queue_index_remove = 0;
			}
			else
			{
				printf("L2CAP connection to device %s failed. status code %u\n", bd_addr_to_str(event_addr), packet[2]);
				exit(1);
			}
			break;

		case L2CAP_EVENT_CHANNEL_CLOSED:
			//channel closed
			printf("L2CAP channel closed");
			break;

		case L2CAP_EVENT_CREDITS:
			l2cap_try_send_queued();
			break;

		default: break;
    	}
    }

    if (packet_type == L2CAP_DATA_PACKET)
    {
    	// protocol function
    	protocol(packet, size, channel);
    	l2cap_hand_out_credits();
    	//printf("source cid %x -- ", channel);
    	//hexdump( packet, size );
    }
}

