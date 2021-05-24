#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "ros_bridge/robot_client/robot_client.hpp"

using namespace std::chrono_literals;

class WebotsController : public rclcpp::Node
{
  public:
    WebotsController()
    : Node("webots_controller") {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);
      timer_ = this->create_wall_timer(
      8ms, std::bind(&WebotsController::timer_callback, this));
      client = new RobotClient("127.0.0.1", 10001, 3);
      client->connectClient();
    }

  private:
    void timer_callback()
    {
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
    RobotClient* client;
};

int main(int argc, char * argv[]) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebotsController>());
  rclcpp::shutdown();
  return 0;
}