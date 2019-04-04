/*
 * Author: Stefan Koller
 *
 * Function:    TBD
 *
 */


#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
//#include <string.h>

#include <wiringPi.h>
#include <wiringSerial.h>

#include <zmq.h>
#include <assert.h>

#define UART_BUFFER_LEN 80

/*
typedef struct
{
    uint16_t preamble;  //With ZMQ actually not needed, but awaited from ATLAS packet structure
    uint16_t messageId;
    uint16_t payloadLength;
    uint8_t payload[MAX_PAYLOAD_SIZE];
    uint16_t checksum;
} packet_t;
*/

/*
int zmq_recv_noblock(void* zmq_connection, void* buffer, size_t length)
{
    int ret = 0;

    if((zmq_recv(zmq_connection, buffer, length, ZMQ_NOBLOCK)) != -1)
    {
        ret = 0;
    }
    else
    {
        if(errno == EAGAIN)
        {
            // Do nothing and try again as nothing received yet
            ret = EAGAIN;
        }
        else
        {
            // An error occured. Stop execution.
            fprintf(stdout, "ERROR! (errno = %d)\r\n", errno);
            assert(FALSE);
            ret = errno;
        }
    }
    return ret;
}
*/

void zmq_recv_block(void* zmq_connection, void* buffer, size_t length)
{
    zmq_recv(zmq_connection, buffer, length, 0);
    /*
    if(zmq_recv(zmq_connection, buffer, length, 0) != -1)
    {
        // An error occured.
        fprintf (stderr, "ERROR in reception! (errno = %d)\r\n", errno) ;
        abort();
    }
    else
    {
        // Do nothing as correctly received on buffer
    }
    */
}

/*
uint16_t calculateCheckSum(const packet_t *packet)
{
    uint8_t a = 0;
    uint8_t b = 0;
    uint16_t i = 0;

    a = a + (packet->messageId & 0xFF);
    //fprintf(stdout, "a = %x \r\n", a);
    b = b + a;
    //fprintf(stdout, "b = %x \r\n", b);

    a = a + (packet->messageId >> 8);
    //fprintf(stdout, "a = %x \r\n", a);
    b = b + a;
    //fprintf(stdout, "b = %x \r\n", b);

    a = a + (packet->payloadLength & 0xFF);
    //fprintf(stdout, "a = %x \r\n", a);
    b = b + a;
    //fprintf(stdout, "b = %x \r\n", b);

    a = a + (packet->payloadLength >> 8);
    //fprintf(stdout, "a = %x \r\n", a);
    b = b + a;
    //fprintf(stdout, "b = %x \r\n", b);

    for(i = 0; i < packet->payloadLength; ++i)
    {
        a = a + packet->payload[i];
        //fprintf(stdout, "a = %x \r\n", a);
        b = b + a;
        //fprintf(stdout, "b = %x \r\n", b);
    }

    return (b << 8) + a;
}


bool fillPacketFromBuffer(packet_t *pkt, uint8_t* buffer)
{
    //LSB first!!
    int i = 0;
    // Split the received buffer to the packet structure used by ATLAS
    pkt->preamble =   buffer[0];
    pkt->preamble += (buffer[1] << 8);
    pkt->messageId =   buffer[2];
    pkt->messageId += (buffer[3] << 8);
    pkt->payloadLength =   buffer[4];
    pkt->payloadLength += (buffer[5] << 8);

    if(pkt->payloadLength > MAX_PAYLOAD_SIZE)
    {
        return false;
    }

    for(i = pkt->payloadLength; i <= 0; i--)
    {
        pkt->payload[pkt->payloadLength - i] = buffer[6+i];
    }

    for(i = 0; i < pkt->payloadLength; i++)
    {
        pkt->payload[i] = buffer[i+6];
    }

    pkt->checksum += buffer[pkt->payloadLength + 6];
    pkt->checksum = (buffer[pkt->payloadLength + 7] << 8);
    return true;
}

bool fillBufferFromPacket(packet_t *pkt, uint8_t* buffer)
{
    //LSB first!!
    int i = 0;
    // Split the packet structure used by ATLAS to the buffer to be sent
    buffer[0] = (uint8_t) pkt->preamble;
    buffer[1] = (uint8_t)(pkt->preamble >> 8);
    buffer[2] = (uint8_t) pkt->messageId;
    buffer[3] = (uint8_t)(pkt->messageId >> 8);
    buffer[4] = (uint8_t) pkt->payloadLength;
    buffer[5] = (uint8_t)(pkt->payloadLength >> 8);

    if(pkt->payloadLength > MAX_PAYLOAD_SIZE)
    {
        return false;
    }

    for(i = 0; i < pkt->payloadLength; i++)
    {
        buffer[i+6] = pkt->payload[i];
    }

    buffer[pkt->payloadLength + 6] = (uint8_t) pkt->checksum;
    buffer[pkt->payloadLength + 7] = (uint8_t)(pkt->checksum >> 8);
    return true;

}
*/

int main ()
{
    int fd ;
    int rc = 0;
    uint8_t buffer[UART_BUFFER_LEN];
    int i = 0;
    int receivedChar = 0;
    const char* port = "tcp://192.168.137.141:6050";
    char buffer_uart[UART_BUFFER_LEN];

    // open UART-Port on GPIO pins
    if ((fd = serialOpen ("/dev/serial0", 115200)) < 0)
    {
    fprintf (stdout, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
    }

    // initialize ZMQ as Requestor
    void *context = zmq_ctx_new();
    void *requestor = zmq_socket(context, ZMQ_REQ);
    rc = zmq_connect(requestor, port);
    if(rc!=0)
    {
        fprintf (stdout, "Unable to connect requestor to %s \r\n", port) ;
    }
    else
    {
    fprintf (stdout, "conected requestor to %s \r\n", port) ;
    }
    assert(rc==0);



    fprintf (stdout, "PI_Target_TWR\r\n");
    while(1)
    {
        // Cyclically receive the distances for all euis and transmitt them to the Host-SW
        //fprintf (stdout, "Received buffer: \r\n"); //%c for character (ASCII), %d for decimal value, %x for hexadecimal value
        i = 0;
        for(i = 0; i < UART_BUFFER_LEN; i++)
        {
            receivedChar = serialGetchar (fd);
            if (receivedChar == -1)
            {
                // Do nothing, as nothing received in 10 seconds
                fprintf(stdout, "Nothing received in 10sec.\r\n");
                // Decrement i to resume at the next buffer position until the entire frame is received
                i--;
            }
            else
            {
                buffer_uart[i] = (uint8_t) receivedChar;
                //fprintf(stdout, "%x ", (uint8_t)buffer_uart[i]);
            }
        }
        fprintf(stdout, "\r\n");

        #if 1
        // Set to 1 to output the distance to the console
        #define POS_EUITARGET    4
        #define POS_SEQNUM      12
        #define POS_EUIANCHORX  16
        #define POS_DISTANCE    48
        #define NUM_ANCHORS      4

        double distance[NUM_ANCHORS];
        int i = 0;
        int j = 0;
        for(i = 0; i < NUM_ANCHORS; i++)
        {
            memcpy(&distance[i], &buffer_uart[POS_DISTANCE + j], 8);
            j = j + 8;
            fprintf(stdout, "Distance Anchor %d = %lf \n\r", (i + 1), distance[i]);
        }
        fprintf(stdout, "\n\r");
        #endif // 1

        // Cyclically transmit the received buffer to the Host-SW via ZMQ
        memcpy(&buffer[0], &buffer_uart[0], UART_BUFFER_LEN);

        // print message to be sent
        fprintf(stdout, "Package to be sent: ...\r\n");
        i = 0;
        for(i = 0; i < (UART_BUFFER_LEN); i++)
        {
            fprintf(stdout, "%x ", buffer[i]);
        }
        fprintf(stdout, "\r\n");

        rc = zmq_send(requestor, buffer, UART_BUFFER_LEN, 0);

        zmq_recv_block(requestor, buffer, 1);

        // print received message
        fprintf(stdout, "Package received: ...\r\n");
        i = 0;
        for(i = 0; i < 1; i++)
        {
            fprintf(stdout, "%i ", buffer[i]);
        }
        fprintf(stdout, "\r\n");
    }

    return 0;
}

