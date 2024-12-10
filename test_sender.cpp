#include <iostream>
#include "udp_socket.h"
#include "OSPFHello.h"

int main() {
    
    UDPSocket socket(0);

    OSPFHello hello = {
        0x00000000, // RouterID
        0xFFFFFF00, // networkMask
        10,         // helloInterval
        1,          // routerPriority
        40,         // deadInterval
        0,          // designatedRouter
        0,          // backupDesignatedRouter
        {0x05060708} // neighbors
    };
    
    socket.sendHello("224.0.0.5", 1050, hello);
    std::cout << "Hello packet sent." << std::endl;

    return 0;
}
