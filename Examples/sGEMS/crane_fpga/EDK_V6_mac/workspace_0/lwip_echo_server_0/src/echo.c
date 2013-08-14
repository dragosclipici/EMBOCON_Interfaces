/*
 * Copyright (c) 2009 Xilinx, Inc.  All rights reserved.
 *
 * Xilinx, Inc.
 * XILINX IS PROVIDING THIS DESIGN, CODE, OR INFORMATION "AS IS" AS A
 * COURTESY TO YOU.  BY PROVIDING THIS DESIGN, CODE, OR INFORMATION AS
 * ONE POSSIBLE   IMPLEMENTATION OF THIS FEATURE, APPLICATION OR
 * STANDARD, XILINX IS MAKING NO REPRESENTATION THAT THIS IMPLEMENTATION
 * IS FREE FROM ANY CLAIMS OF INFRINGEMENT, AND YOU ARE RESPONSIBLE
 * FOR OBTAINING ANY RIGHTS YOU MAY REQUIRE FOR YOUR IMPLEMENTATION.
 * XILINX EXPRESSLY DISCLAIMS ANY WARRANTY WHATSOEVER WITH RESPECT TO
 * THE ADEQUACY OF THE IMPLEMENTATION, INCLUDING BUT NOT LIMITED TO
 * ANY WARRANTIES OR REPRESENTATIONS THAT THIS IMPLEMENTATION IS FREE
 * FROM CLAIMS OF INFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <stdio.h>
#include <string.h>

#include "lwip/err.h"
#include "lwip/udp.h"

#include "controller.h"
#include "FPGAcontroller_library.h"


#ifdef __arm__
extern volatile int TcpFastTimer;
extern volatile int TcpSlowTimer;
#endif
int transfer_data() {
#ifdef __arm__
	if (TcpFastTimer) {
		TcpFastTimer = 0;
		tcp_fasttmr();
	}
	if (TcpSlowTimer) {
		TcpSlowTimer = 0;
		tcp_slowtmr();
	}
#endif
	return 0;
}


/* Define these here so we don't have to keep modifying the code*/
//#define NINPUTS 274
//#define NOUTPUTS 2

#define START_COMPUTING 4294967295
#define START_READING 0


/* UDP_SERVER_FUNCTION
 *
 * This function is the main callback when a UDP packet is received.
 *
 * It copies the payload of the UDP packet to the shared memory
 * interfaces of the CONTROLLER PCORE (write data into REG0), raises a ready flag (write START_COMPUTING into REG2), waits for
 * a completion flag to be returned (wait START_READING from REG2) and read back the results (reads data from REG1)
 * The result is then copied into the payload of a new UDP packet and returned.
 *
 */
void udp_server_function(void *arg, struct udp_pcb *pcb,
		struct pbuf *p, struct ip_addr *addr, u16_t port){

	int k;	// An iterator for loops

	//printf("\n");
	//printf("Received an UDP packet ...\n");
	// Only respond to packets of the correct size
	// (we could make the protocol more complex later on)
	if (p->len == (NINPUTS)*sizeof(unsigned int)){

		// Copy payload into something better aligned
		unsigned int   payload_temp[NINPUTS];
		unsigned char *payload_temp_char = (unsigned char *)payload_temp;

		unsigned char *payload_ptr;
		payload_ptr = (unsigned char *)p->payload;
		for (k=0; k<(NINPUTS)*sizeof(unsigned int); k++){
				payload_temp_char[k] = payload_ptr[k];
		}

		// Variables for shared memory interface


		float *payload_temp_fl = (float *)payload_temp;

		//printf("Writing data ...\n");

		for (k=0; k<2; k++){
			//payload_temp[k]=0;
			//printf("u[%d] = %5.4f\n", k,(float)payload_temp[k]);
			printf("x[%d] = %5.4f\n", k,payload_temp_fl[k]);
		}
		// Copy stuff in REG0
		for (k=0; k<NINPUTS; k++){
		
			//printf("x[%d] = %5.4f\n", k,payload_temp_fl[k]);
			CONTROLLER_mWriteReg(0x77200000, 0, payload_temp[k]); //writing to REG0
		}



		CONTROLLER_mWriteReg(0x77200000, 2*4, START_COMPUTING); //writing to REG2


		/* Poll the output ready register (REG2) until it becomes unity*/
		//printf("Waiting data ready ...\n");
		Xuint32 Data_Reg2;
		while(1){
			Data_Reg2=CONTROLLER_mReadReg(0x77200000, 2*4); //reading from REG2
			//printf("Reg2 = %5.4f\n", (float)Data_Reg2);
			if (Data_Reg2 == START_READING){
				break;
			}
		}


		//printf("Reading data ...\n");
		Xuint32   payload_read[NOUTPUTS];
		// read stuff from


		for (k=0; k<NOUTPUTS; k++){
			payload_read[k]=CONTROLLER_mReadReg(0x77200000, 1*4);//reading from REG1
			//printf("u[%d] = %5.4f\n", k,(float)payload_read[k]);
		}

		float *payload_read_fl = (float *)payload_read;

	/*	//debug only remove
		payload_read[0]=1065353216; //1 in float
		payload_read[1]=1073741824; //2 in float
*/

		for (k=0; k<NOUTPUTS; k++){
			printf("debug u[%d] = %5.4f\n", k,(float)payload_read_fl[k]);
		}


		// Create a reply
		struct pbuf pnew;


		pnew.next = NULL;
		pnew.payload = payload_read;
		pnew.len = NOUTPUTS*4;
		pnew.type = PBUF_RAM;
		pnew.tot_len = pnew.len;
		pnew.ref = 1;
		pnew.flags = 0;

		udp_sendto(pcb, &pnew, addr, port);
		//printf("Data sent to PC\n");

	}
	pbuf_free(p);
}




int start_application()
{
	struct tcp_pcb *pcb;
	err_t err;
	unsigned port = 2008;

	/* create new TCP PCB structure */
	pcb = udp_new();
	if (!pcb) {
		xil_printf("Error creating PCB. Out of Memory\n\r");
		return -1;
	}

	/* bind to specified @port */
	err = udp_bind(pcb, IP_ADDR_ANY, port);
	if (err != ERR_OK) {
		xil_printf("Unable to bind to port %d: err = %d\n\r", port, err);
		return -2;
	}

	/* Set up callback */
	udp_recv(pcb, udp_server_function, NULL);

	xil_printf("UDP echo server started @ port %d\n\r", port);

	return 0;
}
