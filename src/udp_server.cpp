#include "obu/udp_server.hpp"
udp_server::udp_server(const std::string& serverAddress, int serverPort) {
    // 创建UDP套接字
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ == -1) {
        std::cerr << "Error: Could not create socket." << std::endl;
        exit(EXIT_FAILURE);
    }
    
    // 设置服务器地址
    memset(&server_address_, 0, sizeof(server_address_));
    server_address_.sin_family = AF_INET;
    server_address_.sin_port = htons(serverPort);
    server_address_.sin_addr.s_addr = inet_addr(serverAddress.c_str());

    // // 设置客户端地址
    memset(&client_address, 0, sizeof(client_address));
    client_address.sin_family = AF_INET;
    client_address.sin_port = htons(9886); // 目标端口
    client_address.sin_addr.s_addr = inet_addr("127.0.0.2"); // 目标 IP 地址

    // 将套接字与服务器地址绑定
    if (bind(socket_fd_, (struct sockaddr*)&server_address_, sizeof(server_address_)) == -1) {
        std::cerr << "Error: Bind failed." << std::endl;
        exit(EXIT_FAILURE);
    }
}

udp_server::~udp_server() {
    close(socket_fd_);
}

std::vector<uint8_t> udp_server::receive_data(){
    
    // received_data.clear();
    memset(buffer, 0, sizeof(buffer));
    socklen_t client_address_len_ = sizeof(client_address);
    
    ssize_t bytes_received = recvfrom(socket_fd_, buffer, sizeof(buffer), 0,
                                         (struct sockaddr*)&client_address, &client_address_len_);
    if (bytes_received == -1) {
        std::cerr << "Error: Send failed.bytesReceived:" <<bytes_received<< std::endl;
    }
    
    std::vector<uint8_t> received_data(buffer, buffer + bytes_received);
    return received_data;
}

void udp_server::send_to_client(std::vector<uint8_t> message , size_t len_) {
    ssize_t bytes_sent = sendto(socket_fd_, message.data(),len_, 0,
                               (struct sockaddr*)&client_address, sizeof(client_address));
    if (bytes_sent == -1) {
        std::cerr << "Error: Send failed." << std::endl;
    }
    std::cout<<"bytesSent:"<<bytes_sent<<std::endl;
}
