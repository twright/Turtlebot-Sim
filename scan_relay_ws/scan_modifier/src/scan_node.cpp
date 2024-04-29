#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::placeholders;

class ScanModifierNode : public rclcpp::Node
{
public:
   ScanModifierNode() : Node("scan_modifier")
   {
        subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(), std::bind(&ScanModifierNode::scan_sub_callback, this, _1));
        publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan_safe", rclcpp::SensorDataQoS());

      RCLCPP_INFO(get_logger(), "Scan modifier node started!");
   }

private:
   void scan_sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg)
   {
        // laser_msg->set__scan_time(0.000001);
        RCLCPP_INFO(get_logger(), "Received LaserScan: Time between scans: %f", laser_msg->scan_time);
        publisher_->publish(*laser_msg);
   }

   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
   rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanModifierNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}