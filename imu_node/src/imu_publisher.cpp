#include <memory>
#include <string>
#include <iostream>
#include <iomanip>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "xsens_library/xdpchandler.h"
#include "sensor_msgs/msg/imu.hpp"

using namespace std;
bool scan_flag = false;

class XsensPublisher : public rclcpp::Node
{
    public:
        XsensPublisher()
        : Node("xsens_publisher"), count_(0)
        {
            publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
            timer_ = this->create_wall_timer(
            500ms, std::bind(&XsensPublisher::timer_callback, this));
        }

    private:
        XdpcHandler xdpcHandler;
        void GetData();
        bool scanForBluetoothDevices();
        float deg_to_rad(float deg);

        void timer_callback()
        {
            scan_flag = scanForBluetoothDevices();
            if(scan_flag)
            {
                GetData();
            }
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
        size_t count_;

};

bool XsensPublisher::scanForBluetoothDevices()
{
    if (!xdpcHandler.initialize())
    {
        cout << "[imu] Failed to initialize XdpcHandler." << endl;
        return false;
    }

    xdpcHandler.scanForDots();

    if (xdpcHandler.detectedDots().empty())
    {
        cout << "[imu] No Movella DOT device(s) found." << endl;
        xdpcHandler.cleanup();
        return false;
    }

    xdpcHandler.connectDots();

    if (xdpcHandler.connectedDots().empty())
    {
        cout << "[imu] Could not connect to any Movella DOT device(s). Aborting." << endl;
        xdpcHandler.cleanup();
        return false;
    }

    for (auto const& device : xdpcHandler.connectedDots())
    {
        cout << "[imu] Set recording data rate to 30 Hz" << endl;
        if (device->setOutputRate(30))
            cout << "[imu] Successfully set recording rate to 30Hz" << endl;
        else
            cout << "[imu] Setting recording rate failed!" << endl;

        if (!device->startMeasurement(XsPayloadMode::RateQuantities))
        {
            cout << "[imu] Could not put device into measurement mode. Reason: " << device->lastResultText() << endl;
            continue;
            return false;
        }
    }

    return true;
}

void XsensPublisher::GetData()
{
    int64_t startTime = XsTime::timeStampNow();
    bool orientationResetDone = false;

    while (scan_flag)
    {
        if (xdpcHandler.packetsAvailable())
        {
            cout << "\r";
            for (auto const& device : xdpcHandler.connectedDots())
            {
                XsDataPacket packet = xdpcHandler.getNextPacket(device->bluetoothAddress());

                if (!orientationResetDone && (XsTime::timeStampNow() - startTime) > 4000)
                {
                    for (auto const& device : xdpcHandler.connectedDots())
                    {
                        cout << endl << "[imu] Resetting heading for device " << device->bluetoothAddress() << ": ";
                        if (device->resetOrientation(XRM_Heading))
                            cout << " OK";
                        else
                            cout << " NOK: " << device->lastResultText();
                    }
                    cout << endl;
                    orientationResetDone = true;
                }

                auto imu_msgs = sensor_msgs::msg::Imu();
                imu_msgs.header.stamp = this->get_clock()->now();
                imu_msgs.header.frame_id = 'base_footprint';

                if (packet.containsCalibratedAcceleration())
                {
                    XsVector Acceleration = packet.calibratedAcceleration();     
                    
                    // Create a vector to store the converted float numbers.
                    vector<float> accel_arr(3);

                    // Copy the XsReal data to the float vector.
                    copy(Acceleration.data(), Acceleration.data() + 3, accel_arr.data());
                    
                    

                    imu_msgs.linear_acceleration.x = accel_arr[0];
                    imu_msgs.linear_acceleration.y = accel_arr[1];
                    imu_msgs.linear_acceleration.z = accel_arr[2];
                }	

                if (packet.containsCalibratedGyroscopeData())
                {
                    XsVector AngularVelocity = packet.calibratedGyroscopeData();     
                    
                    // Create a vector to store the converted float numbers.
                    vector<float> float_array(3);

                    // Copy the XsReal data to the float vector.
                    copy(AngularVelocity.data(), AngularVelocity.data() + 3, float_array.data());

                    float w_x = deg_to_rad(float_array[0]);
                    float w_y = deg_to_rad(float_array[1]);
                    float w_z = deg_to_rad(float_array[2]);

                    if ((w_z > -0.004) && (w_z < 0.004))
                        w_z = 0;
                    if ((w_y > -0.03) && (w_y < 0.03))
                        w_y = 0;
                    if ((w_x > -0.05) && (w_x < 0.05))
                        w_x = 0;

                    imu_msgs.angular_velocity.x = w_x;
                    imu_msgs.angular_velocity.y = w_y;
                    imu_msgs.angular_velocity.z = w_z;
                }

                publisher_->publish(imu_msgs);
            }
        }   
    
        XsTime::msleep(0);
    }
    cout << endl;

    cout << "[imu] Stopping measurement..." << endl;
    for (auto device : xdpcHandler.connectedDots())
    {
        if (!device->stopMeasurement())
             cout << "[imu] Failed to stop measurement.";
    }
    xdpcHandler.cleanup();
    rclcpp::shutdown();

}

float XsensPublisher::deg_to_rad(float deg)
{
    return deg*M_PI/180.0;
}

void signalHandler(int) {
    scan_flag = false;
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  signal(SIGINT, signalHandler);

  rclcpp::spin(std::make_shared<XsensPublisher>());
  rclcpp::shutdown();

  return 0;
}

