// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/map_request.hpp>

using std::placeholders::_1;

class MapChanger : public rclcpp::Node
{
public:
  MapChanger()
  : Node("map_changer")
  {
    RCLCPP_INFO(this->get_logger(), "Starting map_changer node");
    subscription_ = this->create_subscription<rmf_fleet_msgs::msg::FleetState>(
      "/fleet_states", 10, std::bind(&MapChanger::topic_callback, this, _1));
    publisher_ = this->create_publisher<rmf_fleet_msgs::msg::MapRequest>("/map_request", 10);
  }

private:
  void topic_callback(const rmf_fleet_msgs::msg::FleetState::SharedPtr msg) const
  {
    if (msg->name == "turtlebot3")
    {
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->robots[0].name.c_str());
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->robots[0].location.level_name.c_str());
      if (!(msg->robots[0].path.empty())){
        RCLCPP_INFO(this->get_logger(), "Trigger map change for: '%s'", msg->robots[0].name.c_str());
        if (msg->robots[0].path[0].level_name != msg->robots[0].location.level_name)
        {
          RCLCPP_INFO(this->get_logger(), "Trigger map change to: '%s'", msg->robots[0].path[0].level_name.c_str());

          auto message = rmf_fleet_msgs::msg::MapRequest();
          message.fleet_name = msg->name;
          message.robot_name = msg->robots[0].name;
          message.level_name = "MBC_" + msg->robots[0].path[0].level_name;
          message.map_number = 1;

          if (message.level_name != "MBC_"){
            publisher_->publish(message);
          }
        }
      }
    }
    
  }

  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr subscription_;
  rclcpp::Publisher<rmf_fleet_msgs::msg::MapRequest>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapChanger>());
  rclcpp::shutdown();
  return 0;
}
