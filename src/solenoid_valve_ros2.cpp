#include <rclcpp/rclcpp.hpp>
#include <can_plugins2/msg/frame.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <can_utils.hpp>
#include <solenoid_valve_ros2/solenoid_valve_ros2.hpp>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using std::placeholders::_1;

class solenoid_valve_ros2 : public rclcpp::Node
{
private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
    size_t count_;
    int count = 0;
    Valve valveArray;//valveに関する配列
public:
    solenoid_valve_ros2(/* args */);
    void timer_callback();
    void toggle(uint32_t channel,const sensor_msgs::msg::Joy::SharedPtr msg);
    void normal(uint32_t channel,const sensor_msgs::msg::Joy::SharedPtr msg);
    void valvePublish(uint32_t status[7],uint32_t channel);
};

solenoid_valve_ros2::solenoid_valve_ros2(/* args */) : Node("solenoid_valve_ros2"), count_(0)
{       
    publisher_ = this->create_publisher<can_plugins2::msg::Frame>("can_tx", 10);
    subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&solenoid_valve_ros2::joy_callback, this, _1));
    this->declare_parameter("velButton", 2);
    this->declare_parameter("disButton", 1);
    //ツイスト型の調査
    this->declare_parameter("solenoidValveEnable", rclcpp::PARAMETER_INTEGER_ARRAY);
    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("solenoidValveEnable", valveArray.enable)};
    this->set_parameters(all_new_parameters);

    this->declare_parameter("solenoidValveButton", rclcpp::PARAMETER_INTEGER_ARRAY);//電磁弁ボタンのパラメーター
    std::vector<rclcpp::Parameter> valve_button_parameters{rclcpp::Parameter("solenoidValveButton", valveArray.button)};
    this->set_parameters(valve_button_parameters);

    this->declare_parameter("solenoidValveMode", rclcpp::PARAMETER_STRING_ARRAY);//電磁弁のトグルモードの選択
    std::vector<rclcpp::Parameter> valve_mode_parameters{rclcpp::Parameter("solenoidValveMode", valveArray.mode)};
    this->set_parameters(valve_mode_parameters);

    timer_ = this->create_wall_timer(1000ms, std::bind(&solenoid_valve_ros2::timer_callback, this));
    timer_callback();
}

void solenoid_valve_ros2::timer_callback(){
    for(int i=0; i<7; i++){
        valveArray.enable[i] = this->get_parameter("solenoidValveEnable").as_integer_array()[i];
        valveArray.button[i] = this->get_parameter("solenoidValveButton").as_integer_array()[i];
        valveArray.mode[i] = this->get_parameter("solenoidValveMode").as_string_array()[i];
    }
    RCLCPP_INFO(this->get_logger(), "valve_mode %d %d %d %d %d %d %d!", valveArray.enable[0],valveArray.enable[1],valveArray.enable[2],valveArray.enable[3],valveArray.enable[4],valveArray.enable[5],valveArray.enable[6]);
    RCLCPP_INFO(this->get_logger(), "solenoidValveButton %d %d %d %d %d %d %d!", valveArray.button[0],valveArray.button[1],valveArray.button[2],valveArray.button[3],valveArray.button[4],valveArray.button[5],valveArray.button[6]);
    RCLCPP_INFO(this->get_logger(), "solenoidValveMode %s %s %s %s %s %s %s!", valveArray.mode[0].c_str(), valveArray.mode[1].c_str(), valveArray.mode[2].c_str(), valveArray.mode[3].c_str(), valveArray.mode[4].c_str(), valveArray.mode[5].c_str(), valveArray.mode[6].c_str());
}

void solenoid_valve_ros2::valvePublish(uint32_t status[7],uint32_t channel){
  if(status[channel]==1){
    uint32_t a = 1;
    a = a << (channel);
    valveArray.transmit = valveArray.transmit | a;
  }else{
    uint32_t b = 1;
    b = b << (channel);
    valveArray.transmit = valveArray.transmit ^ b;
  }
}

void solenoid_valve_ros2::toggle(uint32_t channel,const sensor_msgs::msg::Joy::SharedPtr msg){
  if(valveArray.mode[channel] == "Toggle"){
    if(msg->buttons[valveArray.button[channel]]==1){
      if(valveArray.preButton[channel]==0){
        if(valveArray.status[channel]==1){
          valveArray.status[channel]=0;
        }else{
          valveArray.status[channel]=1;
        }
        valvePublish(valveArray.status,channel);
        publisher_->publish(get_frame(0x101,static_cast<uint8_t>(valveArray.transmit)));
        valveArray.preButton[channel]=1;
      }
    }else{
      if(valveArray.preButton[channel]==1){
        valveArray.preButton[channel] = 0;
      }
    }
  }
}

void solenoid_valve_ros2::normal(uint32_t channel,const sensor_msgs::msg::Joy::SharedPtr msg){
  if(valveArray.mode[channel] == "Normal"){
    if(msg->buttons[valveArray.button[channel]]==1){
      if(valveArray.countvalve1[channel]==0){
        valveArray.status[channel]=1;
        valvePublish(valveArray.status,channel);
        publisher_->publish(get_frame(0x101,valveArray.status));
        valveArray.countvalve1[channel]=1;
      }
    }else{
      if(valveArray.countvalve1[channel]==1){
        valveArray.status[channel]=0;
        valvePublish(valveArray.status,channel);
        publisher_->publish(get_frame(0x101,valveArray.status));
        valveArray.countvalve1[channel]=0;
      }
    }
  }
}

void solenoid_valve_ros2::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
  for(int i=0;i<7;i++){
    if(valveArray.enable[i]==1){
      toggle(i,msg);
      //normal(i,msg);
    }
  }
  
//    RCLCPP_INFO(this->get_logger(), "I heard:");
    if(msg->buttons[this->get_parameter("velButton").as_int()]==1){
      publisher_->publish(get_frame(0x100, static_cast<uint8_t>(1)));
    }

    if(msg->buttons[this->get_parameter("disButton").as_int()]==1){
      publisher_->publish(get_frame(0x100,static_cast<uint8_t>(0)));
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<solenoid_valve_ros2>());
    rclcpp::shutdown();
    return 0;
}