
//*****************************************************************************
//
// initialising of bluetooth module and handling of connections
//
// two different handlers are needed, one for hci and another one for l2cap
// you can now implement your own protocoll on l2cap (section L2CAP_DATA_PACKET)
//
//*****************************************************************************

//std libraries
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//ti chip specific library
#include <msp430x54x.h>

//bt-stack libraries
#include "bt_control_cc256x.h"
#include "hal_board.h"
#include "hal_compat.h"
#include "hal_usb.h"
#include <btstack/hci_cmds.h>
#include <btstack/run_loop.h>
#include <btstack/sdp_util.h>
#include "hci.h"
#include "l2cap.h"
#include "btstack_memory.h"
#include "remote_device_db.h"
#include "rfcomm.h"
#include "sdp.h"
#include "config.h"

//own libraries
#include "i2c_lib.h"
#include "protocol.h"
#include "library.h"

// maximum l2cap packet size
const uint16_t  mtu = 0xffff;

// bluetooth channel
extern unsigned int chan;

//value of port2 input
unsigned int port2_status;

bd_addr_t event_addr;

// Bluetooth logic
static void l2cap_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
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
    				chan=channel;
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


static void bt_packet_handler (void * connection, uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{

	switch (packet_type)
	{
		case HCI_EVENT_PACKET:
			hexdump( packet, size );
			switch (packet[0]) {

				case BTSTACK_EVENT_POWERON_FAILED:
					printf("HCI Init failed - make sure you have turned off Bluetooth in the System Settings\n");
					exit(1);
					break;

				case BTSTACK_EVENT_STATE:
					// bt stack activated, get started - set local name
					if (packet[2] == HCI_STATE_WORKING) {
                        hci_send_cmd(&hci_write_local_name, "RASP BT");
					}
					break;

				case HCI_EVENT_COMMAND_COMPLETE:
					if (COMMAND_COMPLETE_EVENT(packet, hci_read_bd_addr)){
                        bt_flip_addr(event_addr, &packet[6]);
                        printf("BD-ADDR: %s\n\r", bd_addr_to_str(event_addr));
                        break;
                    }
					if (COMMAND_COMPLETE_EVENT(packet, hci_write_local_name)){
                        hci_discoverable_control(1);
                        break;
                    }
                    break;

				case HCI_EVENT_LINK_KEY_REQUEST:
					// deny link key request
                    printf("Link key request\n\r");
                    bt_flip_addr(event_addr, &packet[2]);
					hci_send_cmd(&hci_link_key_request_negative_reply, &event_addr);
					break;

				case HCI_EVENT_PIN_CODE_REQUEST:
					// inform about pin code request
                    printf("Pin code request using '0943'\n\r");
                    bt_flip_addr(event_addr, &packet[2]);
					hci_send_cmd(&hci_pin_code_request_reply, &event_addr, 4, "0943");
					break;

				case HCI_EVENT_DISCONNECTION_COMPLETE:
					// connection closed -> quit tes app
					printf("Basebank connection closed\n");

					// exit(0);
					break;

                default:
                    break;
			}
            break;

        default:
            break;
	}
}

int  port2_poll(struct data_source *ds)
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



// main
int main(void)
{
    // stop watchdog timer
    WDTCTL = WDTPW + WDTHOLD;

    //Initialize clock and peripherals
    halBoardInit();
    halBoardStartXT1();
    halBoardSetSystemClock(SYSCLK_16MHZ);

    // init debug UART
    halUsbInit();

    // init LEDs
    P1OUT |= LED_1 | LED_2;
    P1DIR |= LED_1 | LED_2;

    /*//init linkLED
    P1OUT &= ~BIT0;
    P1DIR |= BIT0;*/

    //Setup Input Port 2
    initSwitch(2, BIT0);
    initSwitch(2, BIT1);
    initSwitch(2, BIT2);
    initSwitch(2, BIT3);

    //tie port3 and unused pins of port 2
    /*P3OUT = 0;
    P3DIR = 0xFF;
    P3SEL = 0;
    P2OUT &= 0x0F;
    P2DIR |= 0xF0;
    P2SEL &= 0x0F;*/


	/// GET STARTED with BTstack ///
	btstack_memory_init();
    run_loop_init(RUN_LOOP_EMBEDDED);

    //add gpio port 2 to run loop
    // default values
    port2_status = P2IN & 0x0F;

    data_source_t data_src_port2;
    data_src_port2.process=port2_poll;
    data_src_port2.fd = 0;
    run_loop_add_data_source(&data_src_port2);


    // init HCI
	hci_transport_t    * transport = hci_transport_h4_dma_instance();
	bt_control_t       * control   = bt_control_cc256x_instance();
    hci_uart_config_t  * config    = hci_uart_config_cc256x_instance();
    remote_device_db_t * remote_db = (remote_device_db_t *) &remote_device_db_memory;
	hci_init(transport, config, control, remote_db);

    // use eHCILL
    bt_control_cc256x_enable_ehcill(1);

    // init L2CAP
    l2cap_init();
    l2cap_register_packet_handler(bt_packet_handler);
    l2cap_register_service_internal(NULL, l2cap_packet_handler, 0x1001, mtu);

    // ready - enable irq used in h4 task
    __enable_interrupt();

 	// turn on!
	if(hci_power_control(HCI_POWER_ON))
		printf("power on failed");

	//init i2c
	initi2c();

    // go!
    run_loop_execute();

    // happy compiler!
    return 0;
}

