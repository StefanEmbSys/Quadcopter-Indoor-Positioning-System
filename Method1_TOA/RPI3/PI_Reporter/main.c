/*
 * Author: Stefan Koller
 * This Program is the 2nd part for the programm called "UART_continous_TxAndRx" running on the DWM1001 module.
 * It continously receives data (ASCII-Signs from A to Z) on the UART and prints it to the console. Baseed on the received
 * ASCII-Sign, this program answers with the lower letter (as ASCII-Sign) to the DWM1001 module over UART.
 * The baud rate for both devices is 115200.
 *
 * See PI_UART_continousRXAndTX__ZMQ for details of the ZMQ functionality.
 */


#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <wiringPi.h>
#include <wiringSerial.h>

#include <zmq.h>
#include <assert.h>


#define MAXBUFFERLENGTH 200
#define WAIT_FOR_GO_LENGTH 2


int main ()
{
    int i = 0;
    int rc = 0;
    char buffer[MAXBUFFERLENGTH];
    const char* port = "tcp://192.168.0.27:7000";


    // setup ZeroMQ
    void *context = zmq_ctx_new();
    assert(context!=NULL);
    void *subscriber = zmq_socket(context, ZMQ_SUB);
    assert(subscriber!=NULL);
    rc = zmq_connect(subscriber, port);
    assert(rc==0);
    char *filter = "";
    rc = zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, filter, strlen(filter));
    assert(rc==0);


    fprintf (stdout, "PI_Reporter\r\n");

    fprintf(stdout, "Wait blocked for the GO from ATLAS ...\r\n");
    rc = zmq_recv(subscriber, buffer, 2, 0);
    assert(rc != -1);
    for(i = 0; i < WAIT_FOR_GO_LENGTH; i++)
    {
        fprintf(stdout, "%c", buffer[i]);
        buffer[i] = 0;
    }
    fprintf(stdout, "\r\n");

    while(1)
    {
        fprintf(stdout, "waiting blocked for command ...\r\n");
        rc = zmq_recv(subscriber, buffer, 200, 0);
        assert(rc != -1);
        for(i = 0; i < 200; i++)
        {
            fprintf(stdout, "%c", buffer[i]);
            buffer[i] = 0;
        }
        fprintf(stdout, "\r\n\r\n");
    }
    return 0;
}

