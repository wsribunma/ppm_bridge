#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <boost/asio.hpp> 
using namespace boost;
using std::placeholders::_1;
using namespace std::chrono_literals;


union servo_data_t{
  char bytes[12];
  uint16_t data[6];
};

class MinimalSubscriber : public rclcpp::Node
{
  public:
    //Public member method
    MinimalSubscriber()
    : Node("minimal_subscriber"), m_io(), m_port(m_io)
    {
      // Declare Joy Controller
      this->declare_parameter("controller_id", rclcpp::PARAMETER_STRING);
      rclcpp::Parameter controller_param = this->get_parameter("controller_id");
      m_controller_id_param_str = controller_param.as_string();


      m_subscription = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy_throttle", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      a_subscription = this->create_subscription<sensor_msgs::msg::Joy>(
      "auto_joy_throttle", 10, std::bind(&MinimalSubscriber::auto_callback, this, _1));
      m_pub_status = this->create_publisher<std_msgs::msg::String>("status", 10);

      auto get_status =
      [this]() -> void
      {
        memset(m_out_buf, 0, 2048);
        size_t n_read = m_port.read_some(asio::buffer(m_out_buf,2048));
        m_out_buf[n_read] = '\0';
        RCLCPP_INFO(this->get_logger(), "%s", m_out_buf);
      };
      //m_timer  = this->create_wall_timer(100ms, get_status);

      m_port.open("/dev/ttyACM0");
      m_port.set_option(asio::serial_port_base::baud_rate(57600));
      for (int i = 0; i<5; i++){
        m_servo_data.data[i] = 1000;
      }  
    }
    ~MinimalSubscriber(){
        m_port.close();
    }
    //public member attributes

  private:
    // Private member methodsjoy
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "controller id: %s", m_controller_id_param_str.c_str());
      if (m_controller_id_param_str == "taranis"){

        m_servo_data.data[0] = std::clamp(-500 * msg->axes[0] + 1500, 1000.0f, 2000.0f);
        m_servo_data.data[1] = std::clamp(500 * msg->axes[1] + 1500, 1000.0f, 2000.0f);
        m_servo_data.data[2] = std::clamp(-500 * msg->axes[2] + 1500, 1000.0f, 2000.0f);
        m_servo_data.data[3] = std::clamp(500 * msg->axes[3] + 1500, 1000.0f, 2000.0f);
        m_servo_data.data[4] = std::clamp(1000 * msg->axes[4] + 2000, 1000.0f, 2000.0f);
        
        if (msg->axes[5] > 0){
          m_servo_data.data[0] = a_servo_data.data[0];
          m_servo_data.data[1] = a_servo_data.data[1];
          m_servo_data.data[2] = a_servo_data.data[2];
          m_servo_data.data[3] = a_servo_data.data[3];
          m_servo_data.data[4] = a_servo_data.data[4];
        }
      }
      // else, this will work with f310 logitech
      else {
        m_servo_data.data[0] = std::clamp(1000 * msg->axes[1] + 1000, 1000.0f, 2000.0f);
        m_servo_data.data[1] = std::clamp(500 * msg->axes[3] + 1500, 1000.0f, 2000.0f);
        m_servo_data.data[2] = std::clamp(500 * msg->axes[4] + 1500, 1000.0f, 2000.0f);
        m_servo_data.data[3] = std::clamp(500 * msg->axes[0] + 1500, 1000.0f, 2000.0f);
        m_servo_data.data[4] = std::clamp(1000 * msg->axes[2] + 2000, 1000.0f, 2000.0f);
      }

     
      uint16_t cksum = 0;
      for (int i=0;i<5;i++) {
        cksum += m_servo_data.data[i];
      }
      char packet[14];
      uint16_t header = 65535;
      memcpy(packet, &header, 2);
      memcpy(&packet[2], m_servo_data.bytes, 10);
      memcpy(&packet[12], &cksum, 2);
      m_port.write_some(asio::buffer(packet, 14));

      std_msgs::msg::String status;
      char buf[255];
      snprintf(buf, 255,
        "Sent to serial: [%u, %u, %u, %u, %u]",
          m_servo_data.data[0],
          m_servo_data.data[1],
          m_servo_data.data[2],
          m_servo_data.data[3],
          m_servo_data.data[4]);
      status.data = std::string(buf);
      m_pub_status->publish(status);
    }
     void auto_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      a_servo_data.data[0] = std::clamp(-500 * (msg->axes[0]) + 1500, 1000.0f, 2000.0f);
      a_servo_data.data[1] = std::clamp(500 * msg->axes[1] + 1500, 1000.0f, 2000.0f);
      a_servo_data.data[2] = std::clamp(-500 * msg->axes[2] + 1500, 1000.0f, 2000.0f);
      a_servo_data.data[3] = std::clamp(500 * msg->axes[3] + 1500, 1000.0f, 2000.0f);
      a_servo_data.data[4] = 1900;//std::clamp(msg->axes[4] + 1900, 1000.0f, 2000.0f); // should force it into stabilize mode
    }

    // Member attributes
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_subscription;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr a_subscription;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pub_status;
    asio::io_service m_io;
    asio::serial_port m_port;
    servo_data_t m_servo_data;
    servo_data_t a_servo_data;
      // 

    u_int8_t m_out_buf[2048];
    rclcpp::TimerBase::SharedPtr m_timer;
    std::string m_controller_id_param_str;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}