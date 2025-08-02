#include <utility>
#include "bumpandgo/BumpGoNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"



namespace bumpandgo
{

using namespace std::chrono_literals;
using std::placeholders::_1;
BumpGoNode::BumpGoNode()
:Node("bump_go"), state_(FORWARD)
{
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(), // ao usar QoS temos reliable + volatile
    std::bind(&BumpGoNode::scan_callback, this, _1));
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
    timer_ = create_wall_timer(50ms, std::bind(&BumpGoNode::control_cycle, this));
    state_ts_ = now();

}

void BumpGoNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
    last_scan_ = std::move(msg);
}
void
BumpGoNode::control_cycle()
{
  // Do nothing until the first sensor read
  if (last_scan_ == nullptr) {
      RCLCPP_INFO(this->get_logger(), "NULL");

    return;
  }

  geometry_msgs::msg::Twist out_vel;

  switch (state_) {
    case FORWARD:
      RCLCPP_INFO(this->get_logger(), "State Foward");

      out_vel.linear.x = SPEED_LINEAR;
      

      if (check_forward_2_stop()) {
        RCLCPP_INFO(this->get_logger(), "check_forward_2_stop is true");
 

        go_state(STOP);
      }

      if (check_forward_2_back()) {
        RCLCPP_INFO(this->get_logger(), "check_forward_2_back is true");

        go_state(BACK);
      }
      break;
    case BACK:
      RCLCPP_INFO(this->get_logger(), "State Back");

      out_vel.linear.x = -SPEED_LINEAR;

      if (check_back_2_turn()) {
        RCLCPP_INFO(this->get_logger(), "check_back_2_turn is true");

        go_state(TURN);
      }
      break;
    case TURN:
      RCLCPP_INFO(this->get_logger(), "State Turn");

      out_vel.angular.z = SPEED_ANGULAR;

      if (check_turn_2_forward()) {
        RCLCPP_INFO(this->get_logger(), "check_turn_2_forward is true");

        go_state(FORWARD);
      }

      break;
    case STOP:
        RCLCPP_INFO(this->get_logger(), "State Stop");

      if (check_stop_2_forward()) {
        RCLCPP_INFO(this->get_logger(), "check_stop_2_forward is true");

        go_state(FORWARD);
      }
      break;
  }

  vel_pub_->publish(out_vel);
}


void BumpGoNode::go_state(int new_state)
{
    state_ = new_state;
    state_ts_ = now();
}

bool BumpGoNode::check_forward_2_back()
{
  float angle = last_scan_->angle_min;
  float inc = last_scan_->angle_increment;
  size_t n = last_scan_->ranges.size();
    
  for (size_t i = 0; i < n; ++i) {
      if (std::abs(angle) < FORWARD_ANGLE_WIDTH) {  // 0.26 rad ≈ 15 graus
          if (last_scan_->ranges[i] < OBSTACLE_DISTANCE && std::isfinite(last_scan_->ranges[i])) {
              return true;
          }
      }
      angle += inc;
    }

    return false;
}

bool BumpGoNode::check_forward_2_stop()
{
    // Stop if no sensor readings for 1 second
    auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
    return elapsed > SCAN_TIMEOUT;
}

bool BumpGoNode::check_stop_2_forward()
{
  // Going forward if sensor readings are available
  // again
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed < SCAN_TIMEOUT;
}

bool BumpGoNode::check_back_2_turn()
{
    // Going back for 2 seconds
    return (now() - state_ts_) > BACKING_TIME;
}
bool
BumpGoNode::check_turn_2_forward()
{
  // Turning for 2 seconds
  return (now() - state_ts_) > TURNING_TIME;
}

}