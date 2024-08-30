#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <boost/asio.hpp> 
using namespace boost;
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node

{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber"), port(io)
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

      port.open("/dev/ttyACM0");
      port.set_option(asio::serial_port_base::baud_rate(115200));
      
    }
    ~MinimalSubscriber(){
        port.close();
    }

  private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->axes[0]);
    //   asio::write_some(buf);
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    asio::io_service io;
    asio::serial_port port;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}