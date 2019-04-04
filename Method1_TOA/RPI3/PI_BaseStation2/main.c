/*
 * Author: Stefan Koller
 *
 * Function:    BaseStation is the Requestor in the REQ-REP Pattern of ZMQ between BaseStation(s) and ATLAS.
 *              During Initialization:
 *                  BaseStation sends a Request to ATLAS, that it is ready to receive its Configuration.
 *                  ATLAS waits blocked till this information is received and sends the following information via a Reply:
 *                       - config.role:   Role of the BaseStation (CFG_ROLE_ANCHOR)
 *                       - config.eui:    The extended unique identifier for this specific BaseStation
 *                       - config.channel:unused
 *                  SyncNode transmitts the config.role and config.eui information to the DWM1001 over UART.
 *              Endless Loop:
 *                  Once data is received from the DWM1001 over UART, the data is sent to ATLAS using a Request.
 *                  An "OK" response from ATLAS is awaited to complete the transmission.
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

#define MAX_PAYLOAD_SIZE 49
#define MAX_UART_INI_LENGTH 9
#define MAX_UART_TOAFRAME_LENGTH 33
#define WAIT_FOR_GO_LENGTH 2

typedef struct
{
    uint16_t preamble;  //With ZMQ actually not needed, but awaited from ATLAS packet structure
    uint16_t messageId;
    uint16_t payloadLength;
    uint8_t payload[MAX_PAYLOAD_SIZE];
    uint16_t checksum;
} packet_t;


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

/*
    for(i = pkt->payloadLength; i <= 0; i--)
    {
        pkt->payload[pkt->payloadLength - i] = buffer[6+i];
    }
*/
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

int main ()
{
    int fd ;
    int rc = 0;
    uint8_t buffer[256];
    uint8_t configRole = 0;
    uint64_t configEui = 0;
    packet_t packet;
    int i = 0;
    int receivedChar = 0;
    const char* port = "tcp://192.168.0.27:6002";
    char buffer_uart[MAX_PAYLOAD_SIZE];

    void *context2;
    void *subscriber;
    const char* port2 = "tcp://192.168.0.27:7000";


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

    // initialize ZMQ as Subscriber
    context2 = zmq_ctx_new();
    assert(context2!=NULL);
    subscriber = zmq_socket(context2, ZMQ_SUB);
    assert(subscriber!=NULL);
    rc = zmq_connect(subscriber, port2);
    if(rc!=0)
    {
        fprintf (stdout, "Unable to connect subscriber to %s \r\n", port) ;
    }
    else
    {
    fprintf (stdout, "conected subscriber to %s \r\n", port) ;
    }
    assert(rc==0);
    char *filter = "";
    rc = zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, filter, strlen(filter));
    assert(rc==0);


    fprintf (stdout, "PI_BaseStation\r\n");

    // ----------------------------------------------------------------------
    // ----------------------- Receiption of Config -------------------------
    // ----------------------------------------------------------------------

    // Prepare packet for request
    packet.preamble = 0x62B5;
    packet.messageId = 0x0103;
    packet.payloadLength = 0x01;
    packet.payload[0] = 0xAA;
    packet.checksum = calculateCheckSum(&packet);

    if(!fillBufferFromPacket(&packet, buffer))
    {
        fprintf (stdout, "ERROR! - Packet length to be sent is too long.\r\n");
    }


    // print message to be sent
    fprintf(stdout, "Package to be sent: ...\r\n");
    i = 0;
    for(i = 0; i < (packet.payloadLength + 8); i++)
    {
        fprintf(stdout, "%x ", buffer[i]);
    }
    fprintf(stdout, "\r\n");

    rc = zmq_send(requestor, buffer, (packet.payloadLength + 8), 0);

    zmq_recv_block(requestor, buffer, 18);

    // print received message
    fprintf(stdout, "Package received: ...\r\n");
    i = 0;
    for(i = 0; i < 18; i++)
    {
        fprintf(stdout, "%x ", buffer[i]);
    }
    fprintf(stdout, "\r\n");

    if(!fillPacketFromBuffer(&packet, buffer))
    {
        fprintf (stderr, "ERROR! - Received packet length is too long.\r\n");
    }

    // Store received config
    configRole =  packet.payload[0];
    configEui =   packet.payload[5];
    configEui += (packet.payload[6] << 8);
    configEui += (packet.payload[7] << 16);
    configEui += (packet.payload[8] << 24);
    configEui = (configEui << 32);
    configEui +=  packet.payload[1];
    configEui += (packet.payload[2] << 8);
    configEui += (packet.payload[3] << 16);
    configEui += (packet.payload[4] << 24);


    fprintf(stdout, "configRole = %x \r\n", configRole);
    fprintf(stdout, "configEui_low = %x \r\n", (uint32_t)configEui);
    fprintf(stdout, "configEui_high = %x \r\n", (uint32_t)(configEui >> 32));

    // Fill buffer for transmission to DWM1001
    memcpy(&buffer_uart[0], &packet.payload[0], 9);

    fprintf (stdout, "Transmitted buffer: ...\r\n");
    i = 0;
    for(i = 0; i < MAX_UART_INI_LENGTH; i++)
    {
        fprintf(stdout, "%x ", (uint8_t)buffer_uart[i]);
        // Send every position of the buffer by its own. Necessary as data=0x00 would be treated as '\0' by the function serialPuts
        serialPutchar(fd, buffer_uart[i]);
    }
    fprintf(stdout, "\r\n");


    // Wait for receiption of "GO" from ATLAS to indicate the full initialization
    fprintf(stdout, "Waiting for GO from ATLAS ...\r\n");
    rc = zmq_recv(subscriber, buffer, WAIT_FOR_GO_LENGTH, 0);
    assert(rc != -1);
    i = 0;
    for(i = 0; i < WAIT_FOR_GO_LENGTH; i++)
    {
        fprintf(stdout, "%c", buffer[i]);
    }
    fprintf(stdout, "\r\n");

    // Forward GO to DWM1001 module
    memcpy(&buffer_uart[0], &buffer[0], WAIT_FOR_GO_LENGTH);

    fprintf (stdout, "Transmitted buffer: ...\r\n");
    i = 0;
    for(i = 0; i < WAIT_FOR_GO_LENGTH; i++)
    {
        fprintf(stdout, "%c ", (uint8_t)buffer_uart[i]);
        // Send every position of the buffer by its own. Necessary as data=0x00 would be treated as '\0' by the function serialPuts
        serialPutchar(fd, buffer_uart[i]);
    }
    fprintf(stdout, "\r\n");

    while(1)
    {
        // Cyclically receive the TOA information the DWM1001 received over UWB and sent to RPI over UART.
        fprintf (stdout, "Received TOA message: \r\n"); //%c for character (ASCII), %d for decimal value, %x for hexadecimal value
        i = 0;
        for(i = 0; i < MAX_UART_TOAFRAME_LENGTH; i++)
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
                fprintf(stdout, "%x ", (uint8_t)buffer_uart[i]);
            }
        }
        fprintf(stdout, "\r\n");

        // Cyclically transmit the received TOA frame to the ATLAS-System over ZMQ
        // Fill the packet to be transmitted
        packet.preamble = 0x62B5;
        packet.messageId = 0x0101;
        packet.payloadLength = 0x21;
        memcpy(&packet.payload[0], &buffer_uart[0], MAX_UART_TOAFRAME_LENGTH);
        packet.checksum = calculateCheckSum(&packet);

        if(!fillBufferFromPacket(&packet, buffer))
        {
            fprintf (stdout, "ERROR! - Packet length to be sent is too long.\r\n");
        }

        // print message to be sent
        fprintf(stdout, "Package to be sent: ...\r\n");
        i = 0;
        for(i = 0; i < (packet.payloadLength + 8); i++)
        {
            fprintf(stdout, "%x ", buffer[i]);
        }
        fprintf(stdout, "\r\n");

        rc = zmq_send(requestor, buffer, (packet.payloadLength + 8), 0);

        zmq_recv_block(requestor, buffer, 1);

        // print received message
        fprintf(stdout, "Package received: ...\r\n");
        i = 0;
        for(i = 0; i < 1; i++)
        {
            fprintf(stdout, "%x ", buffer[i]);
        }
        fprintf(stdout, "\r\n");
    }

    return 0;
}

