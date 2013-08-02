 
 /*------------------------------------------------------------------------
 *
 * FPGAcontroller_library.c
 *
 * Functions file for libraries called by:
 *      optimizer.c               
 *
 * E. N. Hartley	[enh20@eng.cam.ac.uk]
 * A. Suardi		[a.suardi@imperial.ac.uk]
 *
 * Version 1.0
 *
 * Date 6-Nov-2012
 *-----------------------------------------------------------------------*/

#include "EMBOCON_DataTypes.h"
#include "EMBOCON_OptimizerInterface.h"



/* Standard libraries */
#include<stdio.h>
#include<stdlib.h>
#include<sys/types.h>
#include<string.h>
#include<fcntl.h>


/* Include socket libraries for different platforms */
#ifdef _WIN32
	#include<winsock.h>
	#include<time.h>
	#pragma comment(lib, "wsock32.lib")
#else
	#include<sys/socket.h>
	#include<arpa/inet.h>
	#include<netinet/in.h>
	#include<netdb.h>
	#include<sys/select.h>
	#include<sys/time.h>
    #include<unistd.h>
#endif

#include "simstruc.h"
#include "FPGAcontroller_library.h"
        


/* Return values */
#define RETVAL_OK           0
#define RETVAL_WSA_FAIL     1
#define RETVAL_NO_HOST      2
#define RETVAL_NO_SOCKET    3
#define RETVAL_NO_SEND      4
#define RETVAL_NO_READ      5
#define RETVAL_TIMEOUT      6
#define RETVAL_WRONGSIZE    7

       
#ifdef _WIN32
	WSADATA wsaData;
#endif

int UDPclient_setup(char *strHostname, int port_number, 
        int *socket_handle, struct sockaddr_in *server_address, int *saddr_len){
   
    
    struct hostent *server;		/* Server host entity */
    
    /* If running Windows, WINSOCK needs to be initialised, so do this here */
    #ifdef _WIN32
    	// Initialize Winsock
		if ( (WSAStartup(MAKEWORD(2,2), &wsaData)) != 0) {    		
            return(RETVAL_WSA_FAIL);
		}
	#endif
            
    
    
    /* Populate the host entity for the server */
    server = gethostbyname(strHostname);

    if (server == NULL) {
        #ifdef _WIN32
                WSACleanup();
        #endif
        return(RETVAL_NO_HOST);
    }

    
    /* Create socket with the socket() system call. */
    *socket_handle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (*socket_handle < 0){
        #ifdef _WIN32
                WSACleanup();
        #endif
        return(RETVAL_NO_SOCKET);
    }
    
    
    *saddr_len = sizeof(*server_address);
    
    /* Configure the server address */
    memset( (void *) server_address, 0, *saddr_len);
    port_number = 2008;
    
    
    server_address->sin_family = AF_INET;
    memcpy((char *)&server_address->sin_addr.s_addr,
          (char *)server->h_addr,
          server->h_length);
    server_address->sin_port = htons(port_number);
    return(RETVAL_OK);
    
    
}



/* Close the socket handle that was created */
void UDPclient_close(int *socket_handle){
    #ifdef _WIN32
        closesocket(*socket_handle);
        WSACleanup();
    #else
        close(*socket_handle);
    #endif
}
    
    

/* Create the packet and wait for a reply */
int UDPclient_call(int *socket_handle, struct sockaddr_in *server_address, int *saddr_len,  const float *buf_in, float *buf_out,
        unsigned int ninputs, unsigned int noutputs, int timeout){
    
       
    int k;						// Iterator
    struct timeval tv;			// Select timeout structure
    fd_set Reader;				// Struct for select function
    int retval;					// Return value from calls to functions
    int ss;                     // Select return
	unsigned int i;
 
    /* Set timeout values */
    tv.tv_sec = timeout;
    tv.tv_usec = 0;

    /* Set up FDS */
    FD_ZERO(&Reader);
    FD_SET(*socket_handle, &Reader);

	
	
    /*
     * SEND PACKET
     */
    retval = sendto(*socket_handle, (const float *)buf_in, ninputs*sizeof(float), 0,
                (struct sockaddr *)server_address, *saddr_len);
    if (!retval){
        return(RETVAL_NO_SEND);
    }
    
     
    
    /*
     * Wait for an incoming packet (or time out)
     */
    ss = select(*socket_handle+1, &Reader, NULL, NULL, &tv);
    
    /* If a packet arrives, process it */
    if (ss){
		// printf("packet received\n");
        retval = recvfrom(*socket_handle, (void *)buf_out, noutputs*sizeof(float), 0,
                       (struct sockaddr *)server_address, saddr_len);
    
        if (retval < 0) {
			return(RETVAL_NO_READ);
        }
        
        if (retval != noutputs*sizeof(float)){
			 return(RETVAL_WRONGSIZE);
        }
    } else {
        /* Otherwise, if a timeout occurs, crash */
		return(RETVAL_TIMEOUT);
    }
    
	return(RETVAL_OK);
}



/* UDP client error */
void UDPclient_error(int retval){
    switch(retval){
        case 1:
            printf("ERROR, Winsock failed to initialise");
            break;
        case 2:
            printf("ERROR, no such host");
            break;
        case 3:
            printf("ERROR, failed to initialise socket");
            break;
        case 4:
            printf("ERROR, failed to send UDP packet");
            break;
        case 5:
            printf("ERROR, failed to read UDP packet");
            break;
        case 6:
            printf("ERROR, timeout waiting for UDP packet");
            break;
        case 7:
            printf("ERROR, wrong sized reply");
            break;
        default:
            break;
    }
    
}


