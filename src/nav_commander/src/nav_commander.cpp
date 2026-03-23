#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp" // 仅用于检查服务器状态
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

enum class TacticalState {
  PENDING,
  WORKING,
  RETURNING_HOME
};

class NavCommanderNode : public rclcpp::Node
{
public:
  NavCommanderNode() 
  : Node("nav_commander"), 
    current_state_(TacticalState::PENDING),
    is_system_ready_(false),
    current_hp_(100)
  {
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/red_standard_robot1/goal_pose_raw", 10);

    // 用于检查 Nav2 是否准备就绪（可选，保留原有的逻辑稳定性）
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/red_standard_robot1/navigate_to_pose");

    ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("nav_ready", 10);
    
    hp_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/robot_hp", 10,
      std::bind(&NavCommanderNode::hp_callback, this, std::placeholders::_1));

    setup_poses();

    init_check_timer_ = this->create_wall_timer(
      1s, [this]() { this->check_initialization(); });

    RCLCPP_INFO(this->get_logger(), "Nav Commander (Decision Maker) started.");
  }

private:
  // 修改后的组件
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_; 
  
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr hp_sub_;
  rclcpp::TimerBase::SharedPtr init_check_timer_;

  geometry_msgs::msg::PoseStamped work_goal_pose_;
  geometry_msgs::msg::PoseStamped home_pose_;
  TacticalState current_state_;
  bool is_system_ready_;
  int current_hp_;

  void setup_poses() {
    // 这里的 frame_id 应与你的全局地图框架一致
    home_pose_.header.frame_id = "map";
    home_pose_.pose.position.x = 0.0;
    home_pose_.pose.position.y = 0.0;
    home_pose_.pose.orientation.w = 1.0;

    work_goal_pose_.header.frame_id = "map";
    work_goal_pose_.pose.position.x = 1.5;
    work_goal_pose_.pose.position.y = 3.0;
    work_goal_pose_.pose.orientation.w = 1.0;
  }

  void check_initialization() {
    if (is_system_ready_) return;

    // 虽然不直接调用，但检查 Nav2 状态能确保系统整体可用
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "Waiting for Nav2 stack...");
      return;
    }

    is_system_ready_ = true;
    init_check_timer_->cancel();

    auto ready_msg = std_msgs::msg::Bool();
    ready_msg.data = true;
    ready_pub_->publish(ready_msg);

    evaluate_tactics();
  }

  void hp_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    // if (current_hp_ == msg->data) return;
    
    current_hp_ = msg->data;
    if (is_system_ready_) {
        evaluate_tactics();
    }
  }

  void evaluate_tactics() {
    if (current_hp_ > 100) {
      // if (current_state_ != TacticalState::WORKING) {
        RCLCPP_INFO(this->get_logger(), "Switching to WORKING mode (HP: %d)", current_hp_);
        current_state_ = TacticalState::WORKING;
        publish_goal_to_filter(work_goal_pose_);
      // }
    } else {
      // if (current_state_ != TacticalState::RETURNING_HOME) {
        RCLCPP_WARN(this->get_logger(), "Switching to HOME mode (HP: %d)", current_hp_);
        current_state_ = TacticalState::RETURNING_HOME;
        publish_goal_to_filter(home_pose_);
      // }
    }
  }

  void publish_goal_to_filter(geometry_msgs::msg::PoseStamped target_pose) {
    target_pose.header.stamp = this->now();
    RCLCPP_INFO(this->get_logger(), "Publishing goal to filter: (%.2f, %.2f)", 
                target_pose.pose.position.x, target_pose.pose.position.y);
    goal_pub_->publish(target_pose);
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavCommanderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}