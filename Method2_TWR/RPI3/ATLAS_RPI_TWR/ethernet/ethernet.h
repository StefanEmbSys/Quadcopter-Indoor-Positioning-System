//
//  TBD
//

#ifndef __ETHERNET_INTERFACE_H_
#define __ETHERNET_INTERFACE_H_

#include <string>

#define MAXBUFFERLENGTH 256

class Ethernet
{
public:
    Ethernet(std::string port, int zmq_connection_type);
    ~Ethernet();

    void send(const void* buffer, size_t length);
    int receive(void* buffer, int length);
    int receive_noblock(void* buffer, int length);

private:
    void* m_portfd;
    void* context;
    void* zmq_connection;
    int port_length;
    const char* ip_port;
    std::string port;
};

#endif
