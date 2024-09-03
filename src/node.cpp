#include <memory>

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
      m_subscription = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

      auto get_status =
      [this]() -> void
      {
        memset(m_out_buf, 0, 2048);
        size_t n_read = m_port.read_some(asio::buffer(m_out_buf,2048));
        m_out_buf[n_read] = '\0';
        RCLCPP_INFO(this->get_logger(), "%s", m_out_buf);
      };
      // m_timer  = this->create_wall_timer(1s, get_status);

      m_port.open("/dev/ttyACM0");
      m_port.set_option(asio::serial_port_base::baud_rate(115200));
      for (int i = 0; i<5; i++){
        m_servo_data.data[i] = 1000;
      }      
    }
    ~MinimalSubscriber(){
        m_port.close();
    }
    //public member attributes

  private:
    // Private member methods
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "joy: %f %f %f %f",
        msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3]);
      m_servo_data.data[0] = 1000*msg->axes[1] + 1000;
      m_servo_data.data[1] = 500*msg->axes[3] + 1500;
      m_servo_data.data[2] = 500*msg->axes[4] + 1500;
      m_servo_data.data[3] = 500*msg->axes[0] + 1500;
      m_servo_data.data[4] = 500*msg->axes[2] + 1500;
      m_servo_data.bytes[10] = '\n';
      m_port.write_some(asio::buffer(m_servo_data.bytes, 11));
    }
    
    // Member attributes
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_subscription;
    asio::io_service m_io;
    asio::serial_port m_port;
    servo_data_t m_servo_data;
    u_int8_t m_out_buf[2048];
    rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}