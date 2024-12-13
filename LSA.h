#ifndef LSA_H
#define LSA_H

#include <cstdint>
#include <vector>
#include <cstring>

struct LSA
{
    uint32_t routerID;
    uint32_t sequenceNumber;
    uint16_t age;
    // first is ID and second is cost
    std::vector<std::pair<uint32_t, uint32_t>> links;
    std::vector<uint8_t> serialize() const{
        size_t fixedPartSize = sizeof(LSA) - sizeof(std::vector<std::pair<uint32_t, uint32_t>>);
        size_t linksSize = links.size() * sizeof(std::pair<uint32_t, uint32_t>);
        std::vector<uint8_t> buffer(fixedPartSize + linksSize);
        std::memcpy(buffer.data(), &routerID, fixedPartSize);
        if(!links.empty()){
            std::memcpy(buffer.data() + fixedPartSize, links.data(), linksSize);
        }
        return buffer;
    }
    static LSA deserialize(const std::vector<uint8_t>& buffer){
        size_t fixedPartSize = sizeof(LSA) - sizeof(std::vector<std::pair<uint32_t, uint32_t>>);
        LSA lsa;
        std::memcpy(&lsa.routerID, buffer.data(), fixedPartSize);
        size_t linksCount = (buffer.size() - fixedPartSize) / sizeof(std::pair<uint32_t, uint32_t>);
        lsa.links.resize(linksCount);
        if(!lsa.links.empty()){
            std::memcpy(lsa.links.data(), buffer.data() + fixedPartSize, linksCount * sizeof(std::pair<uint32_t, uint32_t>));
        }
        return lsa;
    }
};

#endif