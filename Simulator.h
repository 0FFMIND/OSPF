#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <random>
#include <iostream>
#include "Logger.h"

class Simulator {
private:
    std::mt19937 generator;
    std::uniform_int_distribution<uint32_t> distribution; 

public:
    Simulator() : generator(std::random_device{}()), distribution(0, 99) {
        Logger::getInstance().log("SIMULATOR", "Simulator initialized with random generator.");
    }
    
    uint32_t generateCost(const uint32_t& routerID) {
        uint32_t cost = distribution(generator);
        Logger::getInstance().log("SIMULATOR", 
            "Neighbor " + std::to_string(routerID) + ": Cost " + std::to_string(cost));
        return cost;
    }
};

#endif
