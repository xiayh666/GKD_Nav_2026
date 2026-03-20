#pragma once
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <cstring>
#include <functional>
#include <thread>

template <typename T>
class SocketServer
{
private:
    int sockfd;
    sockaddr_in serv_addr;
    sockaddr_in cli_addr;
    char buffer[256];
    std::function<void(const T&)> Fcallback;
    std::jthread thread;

    void server_task()
    {
        while (true)
        {
            memset(buffer, 0, sizeof(buffer));
            socklen_t cli_addr_len = sizeof(cli_addr);
            auto n = recvfrom(sockfd, buffer, 256, MSG_WAITALL, (sockaddr *)&cli_addr, &cli_addr_len);
            if (n > 0)
            {
                T pkg{};
                memcpy(&pkg, buffer, n);
                Fcallback(pkg);
            }
        }

    }

public:
    SocketServer(int64_t port_num, const std::function<void(const T&)>& f) : Fcallback(f), thread(&SocketServer<T>::server_task, this)
    {
        printf("start socket server\n");
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            printf("can't open socket\n");
        }
    
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;
        serv_addr.sin_port = htons(port_num);
    
    
        if (bind(sockfd, (sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            printf("can't bind socket fd with port number");
        }
        
    }

    template<typename PKG>
    void send(const PKG& pkg)
    {
        auto n = sendto(
            sockfd,
            (const char *)(&pkg),
            sizeof(pkg),
            MSG_CONFIRM,
            (const struct sockaddr *)&cli_addr,
            sizeof(cli_addr));
    }
};

