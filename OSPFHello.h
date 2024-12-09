#ifndef OSPF_HELLO_H
#define OSPF_HELLO_H

#include <cstdint>
#include <vector>
#include <cstring>


struct OSPFHello{
    uint32_t networkMask;
    uint16_t helloInterval;
    uint16_t routerPriority;
    uint32_t deadInterval;
    uint32_t designatedRouter;
    uint32_t backupDesignatedRouter;
    std::vector<uint32_t> neighbours;

    std::vector<uint8_t> serialize() const {
        size_t fixedPartSize = sizeof(OSPFHello) - sizeof(std::vector<uint32_t>);
        size_t neighborsSize = neighbours.size() * sizeof(uint32_t);
        std::vector<uint8_t> buffer(fixedPartSize + neighborsSize);
        std::memcpy(buffer.data(), &networkMask, fixedPartSize);

        if (!neighbours.empty()) {
            std::memcpy(buffer.data() + fixedPartSize, neighbours.data(), neighborsSize);
        }

        return buffer;
    }

    static OSPFHello deserialize(const std::vector<uint8_t>& buffer) {

        size_t fixedPartSize = sizeof(OSPFHello) - sizeof(std::vector<uint32_t>);
        
        OSPFHello hello;
        std::memcpy(&hello.networkMask, buffer.data(), fixedPartSize);
        size_t neighborsCount = (buffer.size() - fixedPartSize) / sizeof(uint32_t);
        hello.neighbours.resize(neighborsCount);

        if (!hello.neighbours.empty()) {
            std::memcpy(hello.neighbours.data(), buffer.data() + fixedPartSize, neighborsCount * sizeof(uint32_t));
        }
        return hello;
    }

};


#endif
