#include <iostream>
#include "udp_socket.h"
#include "OSPFHello.h"



int main() {

    UDPSocket socket(1050);
    
    socket.joinMulticastGroup("224.0.0.5");

    std::cout << "Waiting for Hello packets..." << std::endl;

    while (true) {
        OSPFHello hello;
        // OSPFHello hello = socket.receiveHello();

        std::cout << "Received Hello packet:" << std::endl;
        std::cout << "  Network Mask: " << std::hex << hello.networkMask << std::endl;
        std::cout << "  RouterID: " << std::hex << hello.routerID << std::endl;
        std::cout << "  Hello Interval: " << std::dec << hello.helloInterval << " seconds" << std::endl;
        std::cout << "  Router Priority: " << std::dec << hello.routerPriority << std::endl;
        std::cout << "  Dead Interval: " << std::dec << hello.deadInterval << " seconds" << std::endl;
        std::cout << "  Designated Router: " << std::dec << hello.designatedRouter << std::endl;
        std::cout << "  Backup Designated Router: " << std::dec <<  hello.backupDesignatedRouter << std::endl;

        std::cout << "  Neighbors:" << std::endl;
        for (const auto& neighbor : hello.neighbours) {
            std::cout << "    " << std::hex << neighbor << std::endl;
        }
    }

    return 0;
}
