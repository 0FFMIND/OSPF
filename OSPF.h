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

class OSPF{

private:
    uint32_t routerID;
    uint16_t helloInterval;
    uint16_t deadInterval;
    std::unordered_map<uint32_t, Neighbour> neighbours;
    std::chrono::steady_clock::time_point lastHelloTime;
    UDPSocket socket;
    bool isRunning;

    std::thread receiveThread;
    std::queue<OSPFHello> helloQueue;
    std::mutex mtx;
    std::condition_variable cv;

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
                    OSPFHello hello = socket.receiveHello();
                {
                    std::unique_lock<std::mutex> lock(mtx);
                    helloQueue.push(hello);
                }
            }catch(const std::exception& e){
                std::cerr << "Error receiving Hello: " << e.what() << std::endl;
            }
        }
    };

    void addNeighbour(const OSPFHello& hello){
        Neighbour n(hello.routerID, hello.routerPriority);
        neighbours.emplace(hello.routerID, n);
        std::cout << "New neighbor initialized with ID: " << hello.routerID << std::endl;
    }

    void handleHello(const OSPFHello& hello){
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
        std::unique_lock<std::mutex> lock(mtx);
        if(!helloQueue.empty()){
            OSPFHello hello = helloQueue.front();
            helloQueue.pop();
            handleHello(hello);
        }
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
            processHelloQueue();
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