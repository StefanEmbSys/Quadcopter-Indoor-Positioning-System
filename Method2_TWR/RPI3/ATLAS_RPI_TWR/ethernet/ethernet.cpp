//
//  TBD
//

#include "ethernet.h"

#include <string>
#include "string.h"
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <zmq.h>
#include <assert.h>


Ethernet::Ethernet(std::string port, int zmq_connection_type)
{
    int fd = 0;
    std::string text;
    this->port = port;
    this->context = zmq_ctx_new();
    this->zmq_connection = zmq_socket(context, zmq_connection_type);
    this->port_length = port.length();
    this->ip_port = port.c_str();

    switch(zmq_connection_type)
    {
        case ZMQ_SUB:
        {
            fd = zmq_connect(this->zmq_connection, this->ip_port);
            char* filter = "";
            fd = zmq_setsockopt(this->zmq_connection, ZMQ_SUBSCRIBE, filter, strlen(filter));
            text = "connect";
            break;
        }
        case ZMQ_REQ:
        {
            fd = zmq_connect(this->zmq_connection, this->ip_port);
            text = "connect";
            break;
        }
        case ZMQ_REP:
        case ZMQ_PUB:
        {
            fd = zmq_bind(this->zmq_connection, this->ip_port);
            text = "bind";
            break;
        }
        default:
        {
            std::cerr << "Error - Invalid zmq_connection type for port: " << port << std::endl;
            assert(false);
            break;
        }
    }

    if (fd != 0)
    {
        std::cerr << "Error - Unable to " << text << " to port: " << port << std::endl;
        assert(false);
    }
    else
    {
        std::cout << "Port " << port << " " << text << "ed in zmq_connection_type " << zmq_connection_type << std::endl;
        m_portfd = this->context;
    }
    usleep(2000);
}

Ethernet::~Ethernet()
{
    std::cout << "Closed Port at " << this->port << std::endl;
    zmq_ctx_destroy(this->context);
}

void Ethernet::send(const void* buffer, size_t length)
{
    int rc = 0;
    rc = zmq_send(this->zmq_connection, buffer, length, 0);
    //std::cout << "errno = " << errno << ", zmq_errno = " << zmq_errno() << std::endl;
    assert(rc != -1);
}

int Ethernet::receive(void* buffer, int length)
{
    int rc = 0;
    rc = zmq_recv(this->zmq_connection, buffer, length, 0);
    assert(rc != -1);
    return rc;
}

int Ethernet::receive_noblock(void* buffer, int length)
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
            // An error occured.
            std::cout << "ERROR in reception! (errno = " << errno << ")" << std::endl;
            ret = errno;
            abort();
        }
    }
    return ret;
}


