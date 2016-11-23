#ifndef MULTICAST_H
#define MULTICAST_H

#define MULTICAST_IP	"224.16.32.111"
#define MULTICAST_PORT	64646
#define TTL				64
#include <iostream>

class Multicastpp
{

public:
    Multicastpp();

    //	*************************
    //  Open Socket
    //
    //	Input:
    //		const char* = interface name {eth0, wlan0, ...}
    //	Output:
    //		int multiSocket = socket descriptor
    //
    int openSocket(std::string interface, int recv_own_data);



    //	*************************
    //  Close Socket
    //
    //  Input:
    //		int multiSocket = socket descriptor
    //
    void closeSocket();



    //	*************************
    //  Send Data
    //
    //  Input:
    //		int multiSocket = socket descriptor
    //		void* data = pointer to buffer with data
    //		int dataSize = number of data bytes in buffer
    //
    int sendData(void* data, int dataSize);



    //	*************************
    //  Receive Data
    //
    //  Input:
    //		int multiSocket = socket descriptor
    //		void* buffer = pointer to buffer
    //		int bufferSize = total size of buffer
    //
    int receiveData(void* buffer, int bufferSize);

    inline int *getSocket() { return &socket_fd; }

private:
    int socket_fd;
    int recvlen;
    std::string ip_base;
    unsigned int agent_id;
    struct sockaddr_in destAddress;
};

#endif // MULTICAST_H
