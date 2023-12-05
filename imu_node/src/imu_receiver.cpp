#include <memory>
#include <string>
#include <iostream>
#include <iomanip>


#include "rclcpp/rclcpp.hpp"
#include "xsens_library/xdpchandler.h"
#include "sensor_msgs/msg/imu.hpp"

using namespace std;

class XsensReceiver : public rclcpp::Node
{
public:
    XsensReceiver()
        : Node("xsens_receiver")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu_data", 10, std::bind(&XsensReceiver::callback, this, std::placeholders::_1));
    }

private:
    float dv_x, dv_y, dv_z, dt;
    float v_x = 0, v_y = 0, v_z = 0;
    uint32_t current_time, last_time = 0;

    void callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        cout << "Acceleration: " << fixed << msg->linear_acceleration.x << 
                " " << msg->linear_acceleration.y << " " << msg->linear_acceleration.z  
             << " | Angular Velocity: " << msg->angular_velocity.x << " " 
                                 << msg->angular_velocity.y << " " << msg->angular_velocity.z << endl;
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XsensReceiver>());
  rclcpp::shutdown();

  return 0;
}
