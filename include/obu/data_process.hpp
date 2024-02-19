#pragma once
#include <iostream>
#include <nlohmann/json.hpp>
#include <cstring>
#include "macro_scope.hpp"
#include <vector>
#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_light.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_signal.hpp>
#include <std_msgs/msg/float32.hpp>
#include "rclcpp/rclcpp.hpp"

using json = nlohmann::json;
struct Message
{
    uint16_t header=0x5AA5;
    uint16_t version=0x2001;
    uint64_t timestamp=0;
    uint16_t device_type=0x0001;
    int device_id=1234567;
    uint8_t data_direction=0x1;
    uint8_t data_type=0x1;
    uint8_t encryption_type=0x0;
    uint32_t reserved=0xffffffff;
    uint16_t message_length=0;
    std::string message_body;
    uint16_t check_code=0x0005;
    Message() = default;
    operator std::vector<uint8_t>() const;
    Message& operator=(const std::vector<uint8_t>& byte_stream);
};

class data_process {
public:
    data_process(rclcpp::Node::SharedPtr node);
    ~data_process();
    // 获取message的消息体
    rclcpp::Node::SharedPtr node_;
    std::string get_string();
    // 将message打包为字节流
    std::vector<uint8_t> packed_data();
    void parse_data(std::vector<uint8_t> received_data);
    
    using TrafficSignalArray = autoware_auto_perception_msgs::msg::TrafficSignalArray;
    using TrafficSignal = autoware_auto_perception_msgs::msg::TrafficSignal;
    using TrafficLight = autoware_auto_perception_msgs::msg::TrafficLight;
    using Float32 = std_msgs::msg::Float32;
    void callback_heading(const Float32::SharedPtr msg);
    int Lanlet_id = 0;

    rclcpp::Publisher<TrafficSignalArray>::SharedPtr traffic_signal_array_pub_;
    rclcpp::Subscription<Float32>::SharedPtr heading_sub_;

    Message receive_message;
    TrafficSignalArray traffic_signal_array_msg_;
    TrafficSignal traffic_signal_msg_;
    TrafficLight traffic_light_msg_;
    
private:
    json message_json;
};

