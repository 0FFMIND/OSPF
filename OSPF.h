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
#include <unordered_set>

#include "udp_socket.h"
#include "Neighbour.h"
#include "OSPFHello.h"
#include "LSA.h"
#include "Simulator.h"
#include "Logger.h"

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
    std::vector<uint32_t> targets;
    UDPSocket socket;
    bool isRunning;
    Simulator s;

    std::thread receiveThread;
    std::queue<LSA> LSAQueue;
    std::queue<OSPFHello> helloQueue;
    std::mutex mtxHello;
    std::mutex mtxLSA;

    void sendHello(){
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - lastHelloTime).count();
        if(elapsed < helloInterval) return;
        std::vector<uint32_t> n;
        for(const auto& [routerID, routerINFO] : neighbours){
            if(routerINFO.getState() != "Down"){
                n.push_back(routerID);
            }
        }
        OSPFHello hello = {
            0xFFFFFF00, routerID, helloInterval, 1, deadInterval, 0, 0, n
        };
        for(const auto& target : targets){
            socket.sendHello("224.0.0.5", 1050 + target, hello);
        }
        lastHelloTime = now;
        Logger::getInstance().log("OSPF", "Hello packet sent.");
    };

    void monitorNeighbours(){
        auto now = std::chrono::steady_clock::now();
        for(auto& [routerid, neighbour] : neighbours){
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - neighbour.lastHelloTime).count();
            if(elapsed > deadInterval && neighbour.getState() != "Down"){
                neighbour.updateState("Down");
                Logger::getInstance().log("OSPF", "Neighbor " + std::to_string(routerid) + " is unreachable.");
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
                std::vector<uint8_t> buffer(data.begin(), data.end());
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
        Logger::getInstance().log("OSPF", "New neighbor initialized with ID: " + std::to_string(hello.routerID));
    }

    void handleHello(const OSPFHello& hello){
        if(hello.routerID == routerID) return;
        Logger::getInstance().log("OSPF", "Processed Hello from Router ID: " + std::to_string(hello.routerID));
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
        Logger::getInstance().log("OSPF", "Processed and flooded LSA from Router ID: " + std::to_string(lsa.routerID));
        if(lsdb.count(lsa.routerID)){
            LSA l = lsdb[lsa.routerID];
            if(l.sequenceNumber >= lsa.sequenceNumber){
                Logger::getInstance().log("OSPF", "Discarding outdated LSA from Router "+ std::to_string(lsa.routerID));
                return;
            }
        }
        lsdb[lsa.routerID] = lsa;
        for(auto const& [nID, nINFO] : neighbours){
            if(nID == routerID) continue;
            socket.sendLSA("224.0.0.5", 1050 + nID, lsa);
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
        LSA lsa;
        lsa.routerID = routerID;
        lsa.sequenceNumber = lsdb.count(routerID) ? lsdb[routerID].sequenceNumber + 1 : 1;
        lsa.age = 0;
        for(const auto& [nID, nInfo] : neighbours){
            uint32_t nCost = s.generateCost(nID);
            lsa.links.emplace_back(nID, nCost);
        }
        lsdb[lsa.routerID] = lsa;
        for(const auto& target : targets){
            socket.sendLSA("224.0.0.5", 1050 + target, lsa);
        }
        lastLSATime = now;
        Logger::getInstance().log("OSPF", "LSA packet sent.");
    };

    // print all route
    void Dijkstra(){
        auto cmp = [](const auto& a, const auto& b){
            return a.first > b.first;
        };
        // current - prev
        std::unordered_map<uint32_t, uint32_t> previous;
        // currentID - cost
        std::unordered_map<uint32_t, uint32_t> distance;
        std::unordered_set<uint32_t> visited;
        // cost - currentID
        std::priority_queue<std::pair<uint32_t, uint32_t>, std::vector<std::pair<uint32_t, uint32_t>>, decltype(cmp)> pq(cmp);
        for(const auto& [routerID, routerINFO] : lsdb){
            distance[routerID] = UINT32_MAX;
        }
        distance[routerID] = 0;
        pq.push({0,routerID});
        while(!pq.empty()){
            auto [currentCost, currentID] = pq.top();
            pq.pop();
            if(!visited.count(currentID)){
                visited.insert(currentID);
            }else{
                continue;
            }
            LSA lsa = lsdb[currentID];
            for(const auto& [neighbourID, neighbourCost] : lsa.links){
                if(distance[neighbourID] > neighbourCost + currentCost){
                    distance[neighbourID] = neighbourCost + currentCost;
                    previous[neighbourID] = currentID;
                    pq.push({distance[neighbourID], neighbourID});
                }
            }
        }

        Logger& logger = Logger::getInstance();
        for (const auto& [dest, cost] : distance) {
            if (cost == UINT32_MAX) {
                logger.log("DIJKSTRA", "Destination " + std::to_string(dest) + " is unreachable.");
                continue;
            }
            std::vector<uint32_t> path;
            for (uint32_t at = dest; at != routerID; at = previous[at]) {
                path.push_back(at);
            }
            path.push_back(routerID);
            std::reverse(path.begin(), path.end());

            std::string pathStr = "Router " + std::to_string(routerID) + " -> Router " + std::to_string(dest) +
                              ", Cost: " + std::to_string(cost) + ", Path: ";
            for (const auto& node : path) {
                pathStr += std::to_string(node) + " ";
            }
            logger.log("DIJKSTRA", pathStr);
        }
    }

    void Dijkstra(const uint32_t& destination){
        if (destination == routerID) {
            Logger::getInstance().log("DIJKSTRA", "Destination is the current router.");
            return;
        }
        auto cmp = [](const auto& a, const auto& b){
            return a.first > b.first;
        };
        // first is cost, second is routerID
        std::priority_queue<std::pair<uint32_t, uint32_t>, std::vector<std::pair<uint32_t, uint32_t>>, decltype(cmp)> pq(cmp);
        std::unordered_map<uint32_t, uint32_t> distance;
        std::unordered_map<uint32_t, uint32_t> previous;
        std::unordered_set<uint32_t> visited;

        for(const auto& [routerID, routerINFO] : lsdb){
            distance[routerID] = UINT32_MAX;
        }

        distance[routerID] = 0;
        pq.push({routerID, 0});
        while(!pq.empty()){
            uint32_t currentID = pq.top().second;
            if(currentID == destination){
                break;
            }
            uint32_t currentCost = pq.top().first;
            pq.pop();
            if(!visited.count(currentID)){
                visited.insert(currentID);
            }else{
                continue;
            }
            LSA lsa = lsdb[currentID];
            for(const auto& link : lsa.links){
                if(link.second + currentCost < distance[link.first]){
                    distance[link.first] = link.second +currentCost;
                    previous[link.first] = currentID;
                    pq.push({link.first, distance[link.first]});
                }
            }
        }
        
        if(distance[destination] == UINT32_MAX){
            Logger::getInstance().log("DIJKSTRA", "Destination " + std::to_string(destination) + " is unreachable.");
        }
        std::vector<uint32_t> path;
        for (uint32_t at = destination; at != routerID; at = previous[at]) {
            path.push_back(at);
        }
        path.push_back(routerID);
        std::reverse(path.begin(), path.end());

        std::string pathStr = "Router " + std::to_string(routerID) + " -> Router " + std::to_string(destination) +
                          ", Cost: " + std::to_string(distance[destination]) + ", Path: ";
        for (const auto& node : path) {
            pathStr += std::to_string(node) + " ";
        }
        Logger::getInstance().log("DIJKSTRA", pathStr);
    };

public:

    OSPF(uint32_t id, uint16_t hello_interval, uint16_t dead_interval, uint16_t port) : routerID(id), helloInterval(hello_interval), deadInterval(dead_interval), socket(port), isRunning(true){
        Logger::getInstance().log("OSPF", "OSPF initialized with Router ID: " + std::to_string(id));
        socket.joinMulticastGroup("224.0.0.5");
    };

    ~OSPF(){
        stop();
        if(receiveThread.joinable()){
            receiveThread.join();
        }
    };

    void setTargets(const std::vector<uint32_t>& target){
        targets = target;
    }

    void start(){
        Logger::getInstance().log("OSPF", "Starting OSPF...");

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
        Logger::getInstance().log("OSPF", "Stopping OSPF...");
    };

    void printStatus() {
        Logger& logger = Logger::getInstance();        
        logger.log("ROUTER", "Router ID: " + std::to_string(routerID));
        logger.log("ROUTER", "Neighbors:");
        for (const auto& [id, neighbor] : neighbours) {
            logger.log("ROUTER", "  Neighbor ID: " + std::to_string(id) + ", State: " + neighbor.getState());
        }

        logger.log("ROUTER", "LSDB:");
        for (const auto& [id, lsa] : lsdb) {
            std::string lsaInfo = "  LSA from Router " + std::to_string(id) + ": ";
            for (const auto& link : lsa.links) {
                lsaInfo += "(" + std::to_string(link.first) + ", Cost: " + std::to_string(link.second) + ") ";
            }
        logger.log("ROUTER", lsaInfo);
        }   
        Dijkstra();
    }
};

#endif