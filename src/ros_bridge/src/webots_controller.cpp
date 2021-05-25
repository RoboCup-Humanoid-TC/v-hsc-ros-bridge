#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/string.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "cv_bridge/cv_bridge.h"

#include "ros_bridge/robot_client/robot_client.hpp"

#include <json/value.h>
#include <jsoncpp/json/json.h>
#include <fstream>

using namespace std::chrono_literals;

class WebotsController : public rclcpp::Node
{
  public:
    WebotsController()
    : Node("webots_controller") {
      clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);
      image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
      sensor_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("sensor", 10);

      timer_ = this->create_wall_timer(
      8ms, std::bind(&WebotsController::timer_callback, this));
      client = new RobotClient("127.0.0.1", 10001, 3);
      client->connectClient();

      ActuatorRequests request;
      
      // Enable camera
      SensorTimeStep *camera_sensor = request.add_sensor_time_steps();
      camera_sensor->set_name("Camera");
      camera_sensor->set_timestep(16);
      
      Json::Value motors_json;
      std::ifstream json_file("src/ros_bridge/src/motors.json");
      json_file >> motors_json;

      // Enable sensors
      for (unsigned int i=0; i < motors_json.size(); i++) {
        // std::cout<< motors_json[i]["name"].asString() << std::endl;
        // std::cout<< motors_json[i]["time_step"].asDouble() << std::endl;
        SensorTimeStep *sensor = request.add_sensor_time_steps();
        sensor->set_name(motors_json[i]["name"].asString() + "S");
        sensor->set_timestep(motors_json[i]["time_step"].asDouble());
      }

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
          auto clk = rosgraph_msgs::msg::Clock();
          clk.clock = rclcpp::Time(sensors.time());
          // std::cout<< clk.clock.seconds() << std::endl;
          clock_publisher_->publish(clk);
          publishImage(sensors);
          publishSensors(sensors);
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

    void publishSensors(const SensorMeasurements& sensors){
      auto jointmsg = sensor_msgs::msg::JointState();
      for (int i=0; i < sensors.position_sensors_size(); i++) {
        jointmsg.name.push_back(sensors.position_sensors(i).name());
        jointmsg.position.push_back(sensors.position_sensors(i).value());
      }
      sensor_publisher_->publish(jointmsg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sensor_publisher_;
    RobotClient* client;
};

int main(int argc, char * argv[]) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebotsController>());
  rclcpp::shutdown();
  return 0;
}