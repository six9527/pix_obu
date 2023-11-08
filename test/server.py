import socket

# 创建 UDP 套接字
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 绑定服务器地址和端口
server_address = ('127.0.0.2', 9886)  # 服务器地址和端口
server_socket.bind(server_address)

while True:
    print("等待接收数据...")
    data, client_address = server_socket.recvfrom(2048)  # 接收数据，最大字节长度为 1024
    print(f"从 {client_address} 收到消息: {data}")

    # 发送响应
    response = "服务器已收到消息"
    server_socket.sendto(data, client_address)
