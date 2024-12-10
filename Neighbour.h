#ifndef NEIGHBOUR_H
#define NEIGHBOUR_H

#include <iostream>
#include <chrono>

enum Neighbour_State{
    Down,
    Init,
    TwoWay,
    Full,
};

class Neighbour{
private:
    uint32_t routerID;
    Neighbour_State state;
    uint16_t priority;

    static std::string stateToString(Neighbour_State state){
        switch (state)
        {
            case Down: return "Down";
            case Init: return "Init";
            case TwoWay: return "TwoWay";
            case Full: return "Full";
            default: return "Unknown";
        }
    };

    static Neighbour_State pharseState(const std::string& state){
        if(state == "Down") return Down;
        if(state == "Init") return Init;
        if(state == "TwoWay") return TwoWay;
        if(state == "Full") return Full;
        throw std::invalid_argument("Invalid state string: " + state);
    };

public:
    std::chrono::steady_clock::time_point lastHelloTime;

    Neighbour(uint32_t id, uint16_t prio) : routerID(id), state(Down), priority(prio), lastHelloTime(std::chrono::steady_clock::now()){}

    Neighbour() = default;
    
    std::string getState() const{
        return stateToString(state);
    };

    void updateState(const std::string& newState){
        Neighbour_State newstate = pharseState(newState);
        if(newstate != state){
            state = newstate;
            std::cout << "Neighbour " << routerID << " state changed to " << stateToString(state) << std::endl;
        }
    };

};

#endif