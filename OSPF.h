#ifndef OSPF_H
#define OSPF_H

#include "udp_socket.h"
#include "Neighbour.h"
#include "OSPFHello.h"

class OSPF{
private:
    uint32_t routerID;
    uint16_t helloInterval;
    uint16_t deadInterval;
    UDPSocket socket;
    bool isRunning;
public:
    OSPF(uint32_t id, uint16_t port) : routerID(id), socket(port), isRunning(true){
        std::cout << "OSPF initialized with Router ID: " << id << std::endl;

    };
    void start(){
        std::cout << "Starting OSPF..." << std::endl;

    };
    void stop(){
        isRunning = false;
        std::cout << "Stopping OSPF..." << std::endl;
    };
};

#endif