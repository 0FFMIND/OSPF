#ifndef OSPF_H
#define OSPF_H

#include <unordered_map>
#include <chrono>
#include <thread>
#include <chrono>
#include <mutex>
#include <queue>
#include <algorithm> 
#include <condition_variable>

#include "udp_socket.h"
#include "Neighbour.h"
#include "OSPFHello.h"
#include "LSA.h"
#include "Simulator.h"

class OSPF{

private:
    uint32_t routerID;
    uint16_t helloInterval;
    uint16_t deadInterval;
    // As neighbour -- routerID - routerInfo(state/priority)
    std::unordered_map<uint32_t, Neighbour> neighbours;
    // routerID - link state(include link route cost)
    std::unordered_map<uint32_t, LSA> lsdb;
    std::chrono::steady_clock::time_point lastHelloTime;
    std::chrono::steady_clock::time_point lastLSATime;
    UDPSocket socket;
    bool isRunning;

    std::thread receiveThread;
    std::queue<LSA> LSAQueue;
    std::queue<OSPFHello> helloQueue;
    std::mutex mtxHello;
    std::mutex mtxLSA;

    void sendHello(){
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - lastHelloTime).count();
        if(elapsed < helloInterval) return;
        OSPFHello hello = {
            0xFFFFFF00, routerID, helloInterval, 1, deadInterval, 0, 0,
            {0x05060708}
        };
        socket.sendHello("224.0.0.5", 1050, hello);
        lastHelloTime = now;
        std::cout << "Hello packet sent." << std::endl;
    };

    void monitorNeighbours(){
        auto now = std::chrono::steady_clock::now();
        for(auto& [routerid, neighbour] : neighbours){
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - neighbour.lastHelloTime).count();
            if(elapsed > deadInterval && neighbour.getState() != "Down"){
                neighbour.updateState("Down");
                std::cout << "Neighbor " << routerID << " is unreachable." << std::endl;
            }
        }
    };

    void receiveThreadWorker(){
        while(isRunning){
            try{
                std::string data = socket.receive();
                if (data.empty()) {
                    throw std::runtime_error("Empty data received");
                }
                std::cout << "Received data size: " << data.size() << " bytes" << std::endl;
                std::vector<uint8_t> buffer(data.begin(), data.end());
                for (const auto& byte : buffer) {
                    std::cout << std::hex << static_cast<int>(byte) << " ";
                }
                std::cout << std::endl;
                packet_type type = static_cast<packet_type>(buffer[0]);
                std::vector<uint8_t> payload(buffer.begin() + 1, buffer.end());
                switch (type) {
                    case OSPFHELLO_PACKET: {
                        OSPFHello hello = OSPFHello::deserialize(payload);
                        std::unique_lock<std::mutex> lock(mtxHello);
                        helloQueue.push(hello);
                        break;
                    }
                    case LSA_PACKET: {
                        LSA lsa = LSA::deserialize(payload);
                        std::unique_lock<std::mutex> lock(mtxLSA);
                        LSAQueue.push(lsa);
                        break;
                    }
                    default:
                        std::cerr << "Unknown packet type: " << static_cast<int>(type) << std::endl;
                        break;
                }
            }catch(const std::exception& e){
                std::cerr << "Error receiving packet: " << e.what() << std::endl;
            }
        }
    };

    void addNeighbour(const OSPFHello& hello){
        Neighbour n(hello.routerID, hello.routerPriority);
        neighbours.emplace(hello.routerID, n);
        std::cout << "New neighbor initialized with ID: " << hello.routerID << std::endl;
    }

    void handleHello(const OSPFHello& hello){
        if(hello.routerID == routerID) return;
        std::cout << "Processing Hello from Router ID: " << hello.routerID << std::endl;
        if(!neighbours.count(hello.routerID)){
            addNeighbour(hello);
        }
        Neighbour& n = neighbours[hello.routerID];
        n.lastHelloTime = std::chrono::steady_clock::now();
        if(n.getState() == "Down"){
            n.updateState("Init");
        }else if(n.getState() == "Init" && std::find(hello.neighbours.begin(), hello.neighbours.end(), routerID) != hello.neighbours.end()){
            n.updateState("TwoWay");
        }
    }

    void processHelloQueue(){
        std::unique_lock<std::mutex> lock(mtxHello);
        if(!helloQueue.empty()){
            OSPFHello hello = helloQueue.front();
            helloQueue.pop();
            handleHello(hello);
        }
    };

    void handleLSA(const LSA& lsa){
        if(lsa.routerID == routerID) return;
        std::cout << "Processing LSA from Router ID: " << lsa.routerID << std::endl;
        if(lsdb.count(lsa.routerID)){
            LSA l = lsdb[lsa.routerID];
            if(l.sequenceNumber >= lsa.sequenceNumber){
                std::cout << "Discarding outdated LSA from Router " << lsa.routerID << std::endl;
                return;
            }
        }
        lsdb[lsa.routerID] = lsa;
        for(auto const& [nID, nINFO] : neighbours){
            if(nID == routerID) continue;
            socket.sendLSA("224.0.0.5", 1050, lsa);
            std::cout << "Flooded LSA from Router " << lsa.routerID << " to Neighbor " << nID << std::endl;
        }
    }

    void processLSAQueue(){
        std::unique_lock<std::mutex> lock(mtxLSA);
        if(!LSAQueue.empty()){
            LSA lsa = LSAQueue.front();
            LSAQueue.pop();
            handleLSA(lsa);
        }
    };

    void sendLSA(){
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - lastLSATime).count();
        if(elapsed < 10 /* LSA Interval*/) return;
        Simulator s;
        LSA lsa;
        lsa.routerID = routerID;
        lsa.sequenceNumber = lsdb.count(routerID) ? lsdb[routerID].sequenceNumber + 1 : 1;
        lsa.age = 0;
        for(const auto& [nID, nInfo] : neighbours){
            uint32_t nCost = s.generateCost(nID);
            lsa.links.emplace_back(nID, nCost);
        }
        lsdb[lsa.routerID] = lsa;
        socket.sendLSA("224.0.0.5", 1050, lsa);
        lastLSATime = now;
        std::cout << "LSA packet sent." << std::endl;
    };

public:

    OSPF(uint32_t id, uint16_t hello_interval, uint16_t dead_interval, uint16_t port) : routerID(id), helloInterval(hello_interval), deadInterval(dead_interval), socket(port), isRunning(true){
        std::cout << "OSPF initialized with Router ID: " << id << std::endl;
        socket.joinMulticastGroup("224.0.0.5");
    };

    ~OSPF(){
        stop();
        if(receiveThread.joinable()){
            receiveThread.join();
        }
    };

    void start(){
        std::cout << "Starting OSPF..." << std::endl;

        receiveThread = std::thread(&OSPF::receiveThreadWorker, this);

        while(isRunning){
            sendHello();
            sendLSA();
            processHelloQueue();
            processLSAQueue();
            monitorNeighbours();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    };

    void stop(){
        isRunning = false;
        std::cout << "Stopping OSPF..." << std::endl;
    };

};

#endif