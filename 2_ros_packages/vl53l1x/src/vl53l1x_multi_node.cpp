#include <cstdlib>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "vl53l1x.h" // Library for the range sensor
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class Vl53l1xPublisher : public rclcpp::Node
{
public:
  Vl53l1xPublisher()
      //: Node("VL53L1X_publisher_node")
      : Node("vl53l1x_node")
  {
    // Declare and get Parameters
    // Set a 500ms timeout on the sensor. (Stop waiting and respond with an error)
    timeout_ = this->declare_parameter("timeout", 1000);
    timing_budget_ = this->declare_parameter("timing_budget", 33000);
    freq_ = this->declare_parameter("frequency", 25.0);
    vl_max_num_ = this->declare_parameter("vl_max_num", 6);
    std::string GPIOstr = "gpioset gpiochip0 ";
    int rp_check = system("cat /etc/hostname | grep 5");
    if (rp_check == 0)
    { // Raspberry pi 5 の場合
      GPIOstr = "gpioset gpiochip4 ";
    }

    for (unsigned int i = 0; i < vl_max_num_; i++) // pin 初期化
    {
      std::cout << GPIOstr + std::to_string(xshutPins[i]) + "=0" << std::endl;
      system((GPIOstr + std::to_string(xshutPins[i]) + "=0").c_str());
    }
    std::this_thread::sleep_for(std::chrono::microseconds(20000));

    for (unsigned int i = 0; i < vl_max_num_; i++)
    {
      // i2cdetect で0x29 が存在するか確認
      system((GPIOstr + std::to_string(xshutPins[i]) + "=1").c_str());
      std::this_thread::sleep_for(std::chrono::microseconds(30000));

      int i2c_exist = system("i2cdetect -y 1 0x29 0x29 | grep 29");
      std::cout
            << "search sensor [" << i << "] " << std::endl;
      if (i2c_exist == 0) // 0x29が存在する場合
      {
        vl_num_++;
        active_sensor.push_back(i);
        std::cout
            << "sensor [" << i << "] exists, set 0x" << std::hex << 0x2A + i << std::endl;
        sensor[i].setTimeout(timeout_);
        if (!sensor[i].init())
        {
          RCLCPP_ERROR(this->get_logger(), "Sensor offline!");
        }

        // Each sensor must have its address changed to a unique value other than
        // the default of 0x29 (except for the last one, which could be left at
        // the default). To make it simple, we'll just count up from 0x2A.
        sensor[i].setAddress(0x2A + i);
        std::this_thread::sleep_for(std::chrono::microseconds(10000)); // Short
        //std::this_thread::sleep_for(std::chrono::microseconds(50000)); // Long
        // std::cout << std::to_string(sensor[i].getAddress()) << std::endl;

        sensor[i].setTimeout(timeout_);
        if (!sensor[i].init())
        {
          RCLCPP_ERROR(this->get_logger(), "Sensor offline!");
        }
        // std::cout << std::to_string(sensor[i].getAddress()) << std::endl;

        // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
        // You can change these settings to adjust the performance of the sensor, but
        // the minimum timing budget is 20 ms for short distance mode and 33 ms for
        // medium and long distance modes. See the VL53L1X datasheet for more
        // information on range and timing limits.
        //sensor[i].setDistanceMode(Vl53l1x::Long);
        sensor[i].setDistanceMode(Vl53l1x::Short);
        sensor[i].setMeasurementTimingBudget(timing_budget_);

        // Start continuous readings at a rate of one measurement every 100 ms (the
        // inter-measurement period). This period should be at least as long as the
        // timing budget.
        sensor[i].startContinuous(static_cast<int>(1000.0 / freq_)); // Hardcode testcase 100
      }
    }
    std::cout << "VL number : " << std::to_string(vl_num_) << std::endl;

    // Setup the publisher
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("~/range", 5);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / freq_)),
        std::bind(&Vl53l1xPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    rclcpp::Time now = this->get_clock()->now();
    if (time0 == 0)
    {
      time0 = now.seconds();
    }

    auto message = std_msgs::msg::Float32MultiArray();
    // message.layout.dim[0].label = "vl53l1x";
    // message.layout.dim[0].size = vl_max_num_;
    // message.layout.dim[0].stride = 1;

    message.data.resize(vl_num_ + 1);
    // message.field_of_view = 0.47; // Typically 27 degrees or 0,471239 radians
    //float range_min = 0.14 * 1000.0; // 140 mm.  (It is actully much less, but this makes sense in the context : Long mode
    //float range_max = 3.00 * 1000.0; // 3.6 m. in the dark, down to 73cm in bright light : Long mode
    float range_min = 0.005 * 1000.0; // 140 mm.  (It is actully much less, but this makes sense in the context
    float range_max = 3.00 * 1000.0; // 3.6 m. in the dark, down to 73cm in bright light
    std::string distances = "";
    for (unsigned int i = 0; i < vl_num_; i++)
    // for (unsigned int i : active_sensor)
    {
      int distance = sensor[active_sensor[i]].read_range();
      if (sensor[active_sensor[i]].timeoutOccurred())
      {
        RCLCPP_ERROR(this->get_logger(), "Timeout Occured!");
        distance = 0;
      }

      if (distance < range_min)
      {
        distance = 0;
      }
      if (distance > range_max)
      {
        distance = 0;
      }
      // distances = distances + "\t vl[" + std::to_string(active_sensor[i]) + "] : " + std::to_string(distance);
      message.data[i] = (float)distance / 1000.0; // range in meters
    }
    message.data[vl_num_] = now.seconds() - time0;
    // std::cout << "time : " << now.seconds() - time0 << "\t " << distances << std::endl;
    //   # (Note: values < range_min or > range_max should be discarded)
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  Vl53l1x sensor[7];
  std::vector<int> active_sensor;
  unsigned int timeout_;
  unsigned int timing_budget_;
  double freq_;
  unsigned int vl_num_ = 0;
  unsigned int vl_max_num_ = 0;
  // int xshutPins[2] = {27, 17};
  int xshutPins[7] = {27, 17, 22,25,10,9,11};
  int time0 = 0;
};  

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vl53l1xPublisher>());
  rclcpp::shutdown();
  return 0;
}
