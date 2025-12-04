// Copyright 2025 CogniPilot Foundation
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

#include <algorithm>
#include <memory>

#include <boost/asio.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

union servo_data_t {
  char bytes[12];
  uint16_t data[6];
};

class PpmBridgeNode : public rclcpp::Node {
public:
  PpmBridgeNode()
  : Node("ppm_bridge_node"),
    m_io(),
    m_port(m_io),
    m_servo_data{},
    a_servo_data{},
    is_auto_mode_(false),
    toggle_mode_switch_(false)
  {
    // Declare parameters
    this->declare_parameter("controller_id", rclcpp::PARAMETER_STRING);
    this->declare_parameter("channel_map", std::vector<int64_t>{0, 1, 2, 3, 4});

    m_controller_id_param_str = this->get_parameter("controller_id").as_string();
    channel_map_ = this->get_parameter("channel_map").as_integer_array();

    m_subscription = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy_throttle", 10, std::bind(&PpmBridgeNode::topic_callback, this, _1));
    a_subscription = this->create_subscription<sensor_msgs::msg::Joy>(
        "auto_joy_throttle", 10, std::bind(&PpmBridgeNode::auto_callback, this, _1));
    m_pub_status = this->create_publisher<std_msgs::msg::String>("status", 10);
    joy_pub_status =
      this->create_publisher<std_msgs::msg::UInt16MultiArray>("joy_serial_status", 10);

    RCLCPP_INFO(this->get_logger(), "Channel map: [%ld, %ld, %ld, %ld, %ld]", channel_map_[0],
                channel_map_[1], channel_map_[2], channel_map_[3], channel_map_[4]);

    // Unused timer callback for debugging serial port status
    [[maybe_unused]] auto get_status = [this]() -> void {
        memset(m_out_buf, 0, 2048);
        size_t n_read = m_port.read_some(boost::asio::buffer(m_out_buf, 2048));
        m_out_buf[n_read] = '\0';
        RCLCPP_INFO(this->get_logger(), "%s", m_out_buf);
      };
    // m_timer  = this->create_wall_timer(100ms, get_status);

    try {
      m_port.open("/dev/ttyACM0");
      m_port.set_option(boost::asio::serial_port_base::baud_rate(57600));
    } catch (const boost::system::system_error & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
      throw;
    }

    for (size_t i = 0; i < 5; i++) {
      m_servo_data.data[i] = 1000;
    }
  }

  ~PpmBridgeNode() {m_port.close();}

private:
  void topic_callback(const sensor_msgs::msg::Joy::ConstSharedPtr & msg)
  {
    RCLCPP_INFO(this->get_logger(), "controller id: %s", m_controller_id_param_str.c_str());

    // Validate message has enough axes and buttons
    if (msg->axes.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty Joy message");
      return;
    }

    if (m_controller_id_param_str == "taranis") {
      if (msg->axes.size() < 6) {
        RCLCPP_WARN(this->get_logger(), "Taranis mode requires at least 6 axes, got %zu",
                    msg->axes.size());
        return;
      }

      m_servo_data.data[0] =
        std::clamp(-500.0 * msg->axes[0] + 1500.0, 1000.0,
                     2000.0);  // joy "+1" is zero throttle , joy "-1" is full throttle
      m_servo_data.data[1] = std::clamp(500.0 * msg->axes[1] + 1500.0, 1000.0,
                                        2000.0);  // joy "+1" is roll left, joy "-1" is roll right
      m_servo_data.data[2] =
        std::clamp(-500.0 * msg->axes[2] + 1500.0, 1000.0,
                     2000.0);  // joy "+1" is elev up (pitch up), joy "-1" elev down (pitch down)
      m_servo_data.data[3] =
        std::clamp(500.0 * msg->axes[3] + 1500.0, 1000.0,
                     2000.0);  // joy "+1" is rudder yaw left, joy "-1 is yaw right"
      m_servo_data.data[4] = std::clamp(1000.0 * msg->axes[4] + 2000.0, 1000.0, 2000.0);

      // Taranis switch-type mode handling
      if (msg->axes[5] > 0) {
        m_servo_data.data[0] = a_servo_data.data[0];
        m_servo_data.data[1] = a_servo_data.data[1];
        m_servo_data.data[2] = a_servo_data.data[2];
        m_servo_data.data[3] = a_servo_data.data[3];
        m_servo_data.data[4] = a_servo_data.data[4];
      }
    } else {
      // else, this will work with f310 logitech
      if (msg->axes.size() < 5 || msg->buttons.empty()) {
        RCLCPP_WARN(this->get_logger(), "Logitech mode requires at least 5 axes and 1 button");
        return;
      }
      // button-type mode handling
      if (msg->buttons[0] == 1 && !toggle_mode_switch_) {
        is_auto_mode_ = !is_auto_mode_;
        toggle_mode_switch_ = true;
      } else if (msg->buttons[0] == 0) {
        toggle_mode_switch_ = false;  // reset button switch when released
      }

      m_servo_data.data[0] = std::clamp(1000.0 * msg->axes[1] + 1000, 1000.0, 2000.0);
      m_servo_data.data[1] = std::clamp(500.0 * msg->axes[3] + 1500, 1000.0, 2000.0);
      m_servo_data.data[2] = std::clamp(500.0 * msg->axes[4] + 1500, 1000.0, 2000.0);
      m_servo_data.data[3] = std::clamp(500.0 * msg->axes[0] + 1500, 1000.0, 2000.0);
      m_servo_data.data[4] =
        std::clamp(1000.0 * msg->axes[2] + 2000, 1000.0, 2000.0);    // stabilizer mode

      // Logitech f310 mode switch using "A" button
      if (is_auto_mode_) {
        m_servo_data.data[0] = a_servo_data.data[0];
        m_servo_data.data[1] = a_servo_data.data[1];
        m_servo_data.data[2] = a_servo_data.data[2];
        m_servo_data.data[3] = a_servo_data.data[3];
        m_servo_data.data[4] = a_servo_data.data[4];
      }
    }

    // Build and send packet
    constexpr size_t NUM_CHANNELS = 5;
    constexpr uint16_t PACKET_HEADER = 0xFFFF;

    uint16_t cksum = 0;
    for (size_t i = 0; i < NUM_CHANNELS; i++) {
      cksum += m_servo_data.data[i];
    }

    uint8_t packet[14];
    memcpy(packet, &PACKET_HEADER, 2);
    memcpy(&packet[2], m_servo_data.bytes, 10);
    memcpy(&packet[12], &cksum, 2);
    m_port.write_some(boost::asio::buffer(packet, 14));

    // Publish status message and array using channel map
    std_msgs::msg::UInt16MultiArray joy_arr;
    std_msgs::msg::String status;

    // Map channels according to channel_map parameter
    joy_arr.data = {m_servo_data.data[channel_map_[0]], m_servo_data.data[channel_map_[1]],
      m_servo_data.data[channel_map_[2]], m_servo_data.data[channel_map_[3]],
      m_servo_data.data[channel_map_[4]]};
    status.data = "Sent to serial: [" + std::to_string(joy_arr.data[0]) + ", " +
      std::to_string(joy_arr.data[1]) + ", " + std::to_string(joy_arr.data[2]) + ", " +
      std::to_string(joy_arr.data[3]) + ", " + std::to_string(joy_arr.data[4]) + "]";

    joy_pub_status->publish(joy_arr);
    m_pub_status->publish(status);
  }

  void auto_callback(const sensor_msgs::msg::Joy::ConstSharedPtr & msg)
  {
    if (msg->axes.size() < 4) {
      RCLCPP_WARN(this->get_logger(), "Auto mode requires at least 4 axes, got %zu",
                  msg->axes.size());
      return;
    }

    // Auto mode input is always in AETR format from controller
    a_servo_data.data[0] = std::clamp(1000.0 * (msg->axes[2]) + 1000, 1000.0,
                                      2000.0);  // Throttle joy[2] from 0 to 1 in percentage
    a_servo_data.data[1] = std::clamp(500.0 * msg->axes[0] + 1500, 1000.0, 2000.0);   // Aileron
    a_servo_data.data[2] = std::clamp(-500.0 * msg->axes[1] + 1500, 1000.0, 2000.0);  // Elevator
    a_servo_data.data[3] = std::clamp(500.0 * msg->axes[3] + 1500, 1000.0, 2000.0);   // Rudder
    a_servo_data.data[4] = 2000.0;  // Force into stabilize mode
  }

  // Member attributes
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_subscription;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr a_subscription;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pub_status;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr joy_pub_status;

  boost::asio::io_service m_io;
  boost::asio::serial_port m_port;
  servo_data_t m_servo_data;
  servo_data_t a_servo_data;

  // Joy button mode handling for Logitech
  bool is_auto_mode_;
  bool toggle_mode_switch_;

  uint8_t m_out_buf[2048];
  rclcpp::TimerBase::SharedPtr m_timer;
  std::string m_controller_id_param_str;
  std::vector<int64_t> channel_map_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PpmBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
