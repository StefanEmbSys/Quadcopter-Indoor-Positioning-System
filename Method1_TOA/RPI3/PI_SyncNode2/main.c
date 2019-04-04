/*
 * Author: Stefan Koller
 *
 * Function:    SyncNode is the Replyer in the REQ-REP Pattern of ZMQ between ATLAS and SyncNode.
 *              During Initialization:
 *                  ATLAS sends the following information via a Request:
 *                       - period.syncPeriod: every 100ms a Sync Frame shall be sent to the BaseStations
 *                       - period.tagPeriod:  not used for SyncNode
 *                  SyncNode receives this request, stores it and replys with "OK" to ATLAS.
 *                  ATLAS sends the following information via a Request:
 *                       - config.role:   Role of the SyncNode (CFG_ROLE_SYNC_ANCHOR)
 *                       - config.eui:    The extended unique identifier for the SyncNode
 *                       - config.channel:unused
 *                  SyncNode receives this request, stores it and replys with "OK" to ATLAS.
 *                  SyncNode transmitts the period.syncPeriod, config.role and config.eui information to the DWM1001 over UART.
 *              Endless Loop:
 *                  Do nothing, as SyncNode operation entirely handled in DWM1001
 *
 */


#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <wiringPi.h>
#include <wiringSerial.h>

#include <zmq.h>
#include <assert.h>

#define payloadSize 49
#define MAX_UART_LENGTH 13
#define WAIT_FOR_GO_LENGTH 2

typedef struct
{
    uint16_t preamble; //With ZMQ actually not needed, but awaited from ATLAS packet structure
    uint16_t messageId;
    uint16_t payloadLength;
    uint8_t payload[payloadSize];
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
    //fprintf(stdout, "a = %i \r\n", a);
    b = b + a;
    //fprintf(stdout, "b = %i \r\n", b);

    a = a + (packet->messageId >> 8);
    //fprintf(stdout, "a = %i \r\n", a);
    b = b + a;
    //fprintf(stdout, "b = %i \r\n", b);

    a = a + (packet->payloadLength & 0xFF);
    //fprintf(stdout, "a = %i \r\n", a);
    b = b + a;
    //fprintf(stdout, "b = %i \r\n", b);

    a = a + (packet->payloadLength >> 8);
    //fprintf(stdout, "a = %i \r\n", a);
    b = b + a;
    //fprintf(stdout, "b = %i \r\n", b);

    for(i = 0; i < packet->payloadLength; ++i)
    {
        a = a + packet->payload[i];
        //fprintf(stdout, "a = %i \r\n", a);
        b = b + a;
        //fprintf(stdout, "b = %i \r\n", b);
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

    if(pkt->payloadLength > payloadSize)
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

    if(pkt->payloadLength > payloadSize)
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
    uint8_t buffer_zmq[256];
    uint32_t syncPeriod = 0;
    uint8_t configRole = 0;
    uint64_t configEui = 0;
    packet_t packet;
    int i = 0;
    char buffer_uart[MAX_UART_LENGTH];
    const char* port  = "tcp://192.168.0.29:6000";
    const char* port2 = "tcp://192.168.0.27:7000";



    // open UART-Port on GPIO pins
    if ((fd = serialOpen ("/dev/serial0", 115200)) < 0)
    {
    fprintf (stdout, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
    }

    // initialize ZMQ as Replyer
    void *context = zmq_ctx_new();
    void *replyer = zmq_socket(context, ZMQ_REP);
    rc = zmq_bind(replyer, port);
    if(rc!=0)
    {
        fprintf (stdout, "Unable to bind replyer to %s\r\n", port) ;
    }
    assert(rc==0);

    // initialize ZMQ as Subscriber
    void *context2 = zmq_ctx_new();
    assert(context2!=NULL);
    void *subscriber = zmq_socket(context2, ZMQ_SUB);
    assert(subscriber!=NULL);
    rc = zmq_connect(subscriber, port2);
    assert(rc==0);
    char *filter = "";
    rc = zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, filter, strlen(filter));
    assert(rc==0);



    fprintf (stdout, "PI_SyncNode\r\n");

    // ----------------------------------------------------------------------
    // ----------------------- Receiption of Period -------------------------
    // ----------------------------------------------------------------------
    zmq_recv_block(replyer, buffer_zmq, 16);

    // print received message
    fprintf(stdout, "Package received: ...\r\n");
    for(i = 0; i < 16; i++)
    {
        fprintf(stdout, "%i ", buffer_zmq[i]);
    }
    fprintf(stdout, "\r\n");

    if(!fillPacketFromBuffer(&packet, buffer_zmq))
    {
        fprintf (stdout, "ERROR! - Received packet length is too long.\r\n");
    }

    // Store received syncPeriod
    syncPeriod =   packet.payload[0];
    syncPeriod += (packet.payload[1] << 8);
    syncPeriod += (packet.payload[2] << 16);
    syncPeriod += (packet.payload[3] << 24);

    fprintf(stdout, "syncPeriod = %d \r\n", syncPeriod);

    // Fill buffer for transmission to DWM1001
    memcpy(&buffer_uart[0], &packet.payload[0], 4);

    // Prepare packet for answer
    packet.payloadLength = 0x01;
    packet.payload[0] = 0x00;
    packet.checksum = calculateCheckSum(&packet);

    if(!fillBufferFromPacket(&packet, buffer_zmq))
    {
        fprintf (stdout, "ERROR! - Packet length to be sent is too long.\r\n");
    }


    // print message to be sent
    fprintf(stdout, "Package to be sent: ...\r\n");
    for(i = 0; i < (packet.payloadLength + 8); i++)
    {
        fprintf(stdout, "%i ", buffer_zmq[i]);
    }
    fprintf(stdout, "\r\n");

    rc = zmq_send(replyer, buffer_zmq, (packet.payloadLength + 8), 0);


    // ----------------------------------------------------------------------
    // ----------------------- Receiption of Config -------------------------
    // ----------------------------------------------------------------------
    zmq_recv_block(replyer, buffer_zmq, 18);

    // print received message
    fprintf(stdout, "Package received: ...\r\n");
    for(i = 0; i < 18; i++)
    {
        fprintf(stdout, "%i ", buffer_zmq[i]);
    }
    fprintf(stdout, "\r\n");

    if(!fillPacketFromBuffer(&packet, buffer_zmq))
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
    memcpy(&buffer_uart[4], &packet.payload[0], 9);

    // Prepare packet for answer
    packet.payloadLength = 1;
    packet.payload[0] = 0x00;
    packet.checksum = calculateCheckSum(&packet);

    if(!fillBufferFromPacket(&packet, buffer_zmq))
    {
        fprintf (stderr, "ERROR! - Packet length to be sent is too long.\r\n");
    }


    // print message to be sent
    fprintf(stdout, "Package to be sent: ...\r\n");
    for(i = 0; i < (packet.payloadLength + 8); i++)
    {
        fprintf(stdout, "%x ", buffer_zmq[i]);
    }
    fprintf(stdout, "\r\n");

    rc = zmq_send(replyer, buffer_zmq, (packet.payloadLength + 8), 0);

    // Fill null terminated string to signalize the end of the transmission on UART
    //buffer_uart[13] = '\0';

    fprintf (stdout, "Transmitted buffer: ...\r\n");
    for(i = 0; i < MAX_UART_LENGTH; i++)
    {
        fprintf(stdout, "%x ", (uint8_t)buffer_uart[i]);
        // Send every position of the buffer by its own. Necessary as data=0x00 would be treated as '\0' by the function serialPuts
        serialPutchar(fd, buffer_uart[i]);
    }
    fprintf(stdout, "\r\n");


    // Wait for the reception of "GO" from ATLAS to indicate the full initialization
    fprintf(stdout, "Wait blocked for the GO from ATLAS ...\r\n");
    rc = zmq_recv(subscriber, buffer_zmq, WAIT_FOR_GO_LENGTH, 0);
    assert(rc != -1);
    for(i = 0; i < WAIT_FOR_GO_LENGTH; i++)
    {
        fprintf(stdout, "%c", buffer_zmq[i]);
    }
    fprintf(stdout, "\r\n");

    memcpy(&buffer_uart[0], &buffer_zmq[0], WAIT_FOR_GO_LENGTH);

    // Transmit the "GO" to the DWM1001 module
    fprintf (stdout, "Transmitted buffer: ...\r\n");
    for(i = 0; i < WAIT_FOR_GO_LENGTH; i++)
    {
        fprintf(stdout, "%c ", (uint8_t)buffer_uart[i]);
        // Send every position of the buffer by its own. Necessary as data=0x00 would be treated as '\0' by the function serialPuts
        serialPutchar(fd, buffer_uart[i]);
    }
    fprintf(stdout, "\r\n");

    while(1)
    {
        // Do nothing as entirely handled in DWM1001 module
    }
    return 0;
}

