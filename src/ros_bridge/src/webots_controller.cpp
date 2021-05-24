#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include "ros_bridge/robot_client/robot_client.hpp"

using namespace std::chrono_literals;

class WebotsController : public rclcpp::Node
{
  public:
    WebotsController()
    : Node("webots_controller") {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);
      image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
      timer_ = this->create_wall_timer(
      8ms, std::bind(&WebotsController::timer_callback, this));
      client = new RobotClient("127.0.0.1", 10001, 3);
      client->connectClient();

      // Enable camera
      ActuatorRequests request;
      SensorTimeStep *camera_sensor = request.add_sensor_time_steps();
      camera_sensor->set_name("Camera");
      camera_sensor->set_timestep(16);
      client->sendRequest(request);
      SensorMeasurements sensors = client->receive();
    }

  private:
    void timer_callback()
    {
      if (client->isOk()) {
        try {
          ActuatorRequests request;
          client->sendRequest(request);
          SensorMeasurements sensors = client->receive();
          publishImage(sensors);
        } catch (const std::runtime_error &exc) {
          std::cerr << "Runtime error: " << exc.what() << std::endl;
        }
      }
    }

    void publishImage(const SensorMeasurements& sensors){
      for (int i=0; i < sensors.cameras_size(); i++) {
        const CameraMeasurement &sensor_data  = sensors.cameras(i);
        if (sensor_data.quality() != -1) {
          throw std::runtime_error("Encoded images are not supported in this client");
        }
        
        cv::Mat img(sensor_data.height(),sensor_data.width(), CV_8UC3, (void*)sensor_data.image().c_str());

        auto imgmsg = sensor_msgs::msg::Image();
        cv_bridge::CvImage img_bridge;
        img_bridge = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, img);
        img_bridge.toImageMsg(imgmsg);
        image_publisher_->publish(imgmsg);
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    RobotClient* client;
};

int main(int argc, char * argv[]) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebotsController>());
  rclcpp::shutdown();
  return 0;
}