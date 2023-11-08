#pragma once

#include <string>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>
class udp_server {
public:
    udp_server(const std::string& serverAddress, int serverPort);
    ~udp_server();
    
    // 启动UDP服务器并开始接收数据
    std::vector<uint8_t> receive_data();
    
    // 发送数据到客户端
    void send_to_client(std::vector<uint8_t> message,size_t len_);

private:
    int socket_fd_;
    char buffer[2048];
    struct sockaddr_in client_address;
    struct sockaddr_in server_address_;
};