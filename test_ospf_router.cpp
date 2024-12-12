#include <iostream>
#include <unordered_map>
#include <cstdint>
#include <vector>

#include "OSPF.h"

int main(void){
    size_t routerCount;
    std::cout << "Enter number of routers: ";
    std::cin >> routerCount;

    std::unordered_map<uint32_t, OSPF*> routers;
    // current routerID - vector neighbour ID
    std::unordered_map<uint32_t, std::vector<uint32_t>> topology;
    
    for(uint32_t i = 1; i <= routerCount; i++){
        routers[i] = new OSPF(i, 10, 40, 1050 + i);
    }
    
    std::cout << "Enter topology (format: routerID neighborID), end with -1 -1 : " << std::endl;
    while(true){
        uint32_t routerID, neighbourID;
        std::cin >> routerID >> neighbourID;
        if (routerID == -1 && neighbourID == -1) {
            break;
        }
        if (routerID < 1 || routerID > routerCount || neighbourID > routerCount || neighbourID < 1) {
            perror("invalid input");
            exit(EXIT_FAILURE);
        }
        topology[routerID].push_back(neighbourID);
        topology[neighbourID].push_back(routerID);
    }

    for(const auto& [routerID, ospf] : routers){
        if(topology.count(routerID)){
            ospf->setTargets(topology[routerID]);
        }
    }

    uint32_t observer;
    std::cout << "Enter the router ID to observe: ";
    std::cin >> observer;

    if (routers.count(observer) == 0) {
        std::cerr << "Invalid router ID!" << std::endl;
        return -1;
    }

    std::vector<std::thread> workers;
    for(const auto& [routerID, ospf] : routers){
        workers.emplace_back([ospf](){
            ospf->start();
        });
    }

    while (true) {
        auto& ospf = *routers[observer];
        ospf.printStatus();
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    for (auto& worker : workers) {
        worker.join();
    }

    for (auto& [routerID, ospf] : routers) {
        delete ospf;
    }

    return 0;
}