#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>

#include "OSPFHello.h"

class UDPSocket{
private:

    int sockfd;
    struct sockaddr_in localAddr;
    struct sockaddr_in remoteAddr;

public:

    UDPSocket(uint16_t loaclPort){
        // create fd
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if(sockfd < 0){
            perror("Failed to create socket");
            exit(EXIT_FAILURE);
        }
        // set localaddr
        memset(&localAddr, 0, sizeof(localAddr));
        localAddr.sin_family = AF_INET;
        localAddr.sin_addr.s_addr = INADDR_ANY;
        localAddr.sin_port = htons(loaclPort);
        // bind localaddr to fd
        if(bind(sockfd, (struct  sockaddr*)&localAddr, sizeof(localAddr)) < 0)
        {
            perror("Failed to bind");
            close(sockfd);
            exit(EXIT_FAILURE);
        };
    }

    ~UDPSocket(){
        close(sockfd);
    }

    void send(const std::string& ip, uint16_t port, const std::string& message){
        memset(&remoteAddr, 0, sizeof(remoteAddr));
        remoteAddr.sin_family = AF_INET;
        remoteAddr.sin_port = htons(port);
        inet_pton(AF_INET, ip.c_str(), &remoteAddr.sin_addr);
        if(sendto(sockfd, message.c_str(), message.size(), 0, (struct sockaddr*)&remoteAddr, sizeof(remoteAddr)) < 0){
            perror("Failed to send data");
        }
    }

    std::string receive(){
        char buffer[1024];
        socklen_t len = sizeof(remoteAddr);
        ssize_t n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&remoteAddr, &len);
        if(n < 0){
            perror("Failed to receive data");
            throw std::runtime_error("Failed");
        }
        return std::string(buffer, n);
    }

    // Hello test
    void sendHello(const std::string& ip, uint16_t port, const OSPFHello& hello){
        std::vector<uint8_t> data = hello.serialize();
        send(ip, port, std::string(data.begin(), data.end()));
    }

    OSPFHello receiveHello() {
        std::string data = receive();
        if (data.empty()) {
            throw std::runtime_error("Empty data received");
        }
        std::cout << "Received data size: " << data.size() << " bytes" << std::endl;
        std::vector<uint8_t> buffer(data.begin(), data.end());
        for (const auto& byte : buffer) {
            std::cout << std::hex << static_cast<int>(byte) << " ";
        }
        std::cout << std::endl;
        return OSPFHello::deserialize(buffer);
    }

    void joinMulticastGroup(const std::string& multicastIP) {
        struct ip_mreq group;
        group.imr_multiaddr.s_addr = inet_addr(multicastIP.c_str());
        group.imr_interface.s_addr = INADDR_ANY;
        if (setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &group, sizeof(group)) < 0) {
            perror("Failed to join multicast group");
            exit(EXIT_FAILURE);
        }
    }
};