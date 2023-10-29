#include <rclcpp/rclcpp.hpp>
#include <can_plugins2/msg/frame.hpp>
#include <sensor_msgs/msg/joy.hpp>


struct Valve{
  uint8_t enable[7];//電磁弁のEnable
  uint32_t status[7];//電磁弁の状態
  std::string mode[7];//ToggleかNomalか
  uint32_t button[7];//ボタン割り当て
  uint32_t transmit = 0;
  uint8_t countvalve1[7];
  uint32_t preButton[7];
};
