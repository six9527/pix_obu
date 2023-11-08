#include "obu/data_process.hpp"
#include "rclcpp/rclcpp.hpp"
#include "obu/udp_server.hpp"

// 创建字节流转化函数
PIXMOVING_SERDE_BS_STRUCT(Message,header,version,timestamp,device_type,device_id,data_direction,data_type,
                                encryption_type,reserved,message_length,message_body,check_code);

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("obu_server_node");
    // 创建UDP服务器实例
    udp_server udp_server("127.0.0.1", 8751);
    data_process data_process_(node);

    while(rclcpp::ok()){
        udp_server.send_to_client(data_process_.packed_data(),29+data_process_.get_string().length());
        data_process_.parse_data(udp_server.receive_data());
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}
