#include "obu/data_process.hpp"
data_process::data_process(rclcpp::Node::SharedPtr node){
    node_ = node;
};
data_process:: ~data_process(){};
std::string data_process::get_string(){
    json sceneData = {
        {"scene", {
            {"fcw", {
                {"vehicleID", 12548456},
                {"position", 0},
                {"distance", 20.0},
                {"alarm_class", 1}
            }},
            {"icw", {
                {"vehicleID", 12548456},
                {"position", 0},
                {"direction", 0},
                {"distance", 20.0},
                {"alarm_class", 1}
            }},
            {"bsw", {
                {"vehicleID", 12548456},
                {"position", 0},
                {"direction", 0},
                {"distance", 20.0},
                {"alarm_class", 1}
            }},
            {"lcw", {
                {"vehicleID", 12548456},
                {"position", 0},
                {"direction", 0},
                {"distance", 20.0},
                {"alarm_class", 1}
            }},
            {"dnpw", {
                {"vehicleID", 12548456},
                {"position", 0},
                {"direction", 0},
                {"distance", 20.0},
                {"alarm_class", 1}
            }},
            {"ebw", {
                {"vehicleID", 12548456},
                {"position", 0},
                {"direction", 0},
                {"distance", 20.0},
                {"alarm_class", 1}
            }},
            {"avw", {
                {"vehicleID", 12548456},
                {"position", 0},
                {"distance", 20.0},
                {"alarm_class", 1}
            }},
            {"clw", {
                {"vehicleID", 12548456},
                {"position", 0},
                {"distance", 20.0},
                {"alarm_class", 1}
            }},
            {"evw", {
                {"vehicleID", 12548456},
                {"position", 0},
                {"hdistance", 10.0},
                {"vdistance", 20.0},
                {"alarm_class", 1}
            }},
            {"lta", {
                {"vehicleID", 12548456},
                {"position", 0},
                {"hdistance", 10.0},
                {"ttp", 0.0},
                {"alarm_class", 1}
            }},
            {"ivs", json::array({
                {
                    {"id", 1},
                    {"sign type", 10},
                    {"description", "注意行人"},
                    {"distance", 120},
                    {"latitude", 0.0},
                    {"longitude", 0.0}
                },
                {
                    {"id", 2},
                    {"sign type", 38},
                    {"description", "施工"},
                    {"distance", 120},
                    {"latitude", 0.0},
                    {"longitude", 0.0}
                }
            })},
            {"tjw", {
                {"congestion_level", 1},
                {"range_latitude", 0.0},
                {"range_longitude", 0.0}
            }},
            {"slw", {
                {"speed", 10.0},
                {"latitude", 0.0},
                {"longitude", 0.0}
            }},
            {"preempt", {
                {"direction", 1}
            }},
            {"glosa", {
                {"latitude", 0.0},
                {"longitude", 0.0},
                {"glosa_info", json::array({
                    {
                        {"state_active", 1},
                        {"state", 3},
                        {"timing", 22},
                        {"maneuvers", 2},
                        {"advisory_active", 1},
                        {"max_speed", 0},
                        {"min_speed", 0}
                    },
                    {
                        {"state_active", 0},
                        {"state", 0},
                        {"timing", 0},
                        {"maneuvers", 0},
                        {"advisory_active", 0},
                        {"max_speed", 0},
                        {"min_speed", 0}
                    }
                })}
            }},
            {"rlvw", {
                {"distance", 120},
                {"maneuvers", 0},
                {"state", 0},
                {"timing", 12},
                {"latitude", 0},
                {"longitude", 0}
            }},
            {"vrucw", {
                {"type", 0},
                {"latitude", 0.0},
                {"longitude", 0.0},
                {"position", 0},
                {"planning_count", 1},
                {"planning_list", json::array({
                    {
                        {"path_planning_count", 1},
                        {"path_planning_points", json::array({
                            {
                                {"latitude", 0.0},
                                {"longitude", 0.0}
                            },
                            {
                                {"latitude", 0.0},
                                {"longitude", 0.0}
                            }
                        })}
                    }
                })}
            }}
        }}
    };
    std::string jsonStr = sceneData.dump();
    return jsonStr;
}

std::vector<uint8_t> data_process::packed_data(){
    Message msg;
    time_t timep;
    std::vector<uint8_t> byte_stream_data;
    std::string msgbody = get_string();
    time(&timep); /*当前time_t类型UTC时间*/
    msg.timestamp = (uint64_t)timep;
    msg.message_length = msgbody.length();
    msg.message_body = msgbody.data();
    msg.check_code = 0x05;
    byte_stream_data = msg;
    // 输出字节流数据
    // show(byte_stream_data);
    return byte_stream_data;
}

void data_process::parse_data(std::vector<uint8_t> received_data){
    Message receive_message;
    TrafficSignalArray traffic_signal_array_msg_;
    TrafficSignal traffic_signal_msg_;
    TrafficLight traffic_light_msg_;
    traffic_signal_array_pub_ = node_->create_publisher<TrafficSignalArray>("/perception/traffic_light_recognition/traffic_signals", rclcpp::QoS(1));

    receive_message = received_data;
    json parsed_json = json::parse(receive_message.message_body);
    std::cout<<"state_active:"<<parsed_json["scene"]["glosa"]["glosa_info"][0]["state_active"]<<std::endl;
    if ((parsed_json["scene"]["glosa"]["glosa_info"][0]["state"]) == 3){
        traffic_light_msg_.color = traffic_light_msg_.GREEN;
        traffic_light_msg_.shape = traffic_light_msg_.LEFT_ARROW;
        traffic_light_msg_.status = traffic_light_msg_.SOLID_ON;
    }
    traffic_light_msg_.confidence = 1;

    traffic_signal_msg_.map_primitive_id = 123456;
    traffic_signal_msg_.lights.push_back(traffic_light_msg_);

    traffic_signal_array_msg_.header.stamp = node_->now();
    traffic_signal_array_msg_.header.frame_id = "obu";
    traffic_signal_array_msg_.signals.push_back(traffic_signal_msg_);
    traffic_signal_array_pub_->publish(traffic_signal_array_msg_);
}