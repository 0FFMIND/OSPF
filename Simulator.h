#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <ctime>
#include <cstdlib>
#include <iostream>

class Simulator{
private:
    /* */
public:
    Simulator(){
        std::srand(std::time(nullptr));
    }
    
    uint32_t generateCost(const uint32_t& routerID){
        uint32_t cost = std::rand() % 100;
        std::cout << "  Neighbor " << std::hex << routerID << ": Cost " << std::dec << cost << std::endl;
        return cost;
    }

};

#endif