#pragma once
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
inline void publish_diagnostic(
    const rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr& publisher,
    const rclcpp::Time& stamp,const std::string& name,int level,const std::string& message,
    std::initializer_list<std::pair<std::string,std::string>> values) {
  diagnostic_msgs::msg::DiagnosticArray output; output.header.stamp=stamp;
  diagnostic_msgs::msg::DiagnosticStatus state;
  state.name=name; state.hardware_id="bof_vision"; state.level=level; state.message=message;
  for(const auto& [key,value]:values) { diagnostic_msgs::msg::KeyValue pair; pair.key=key; pair.value=value; state.values.push_back(pair); }
  output.status.push_back(state); publisher->publish(output);
}
