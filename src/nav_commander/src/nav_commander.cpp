#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"

// 引入 TF2 用于空间初始化自检
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// 机器人的战术状态
enum class TacticalState {
  PENDING,        // 尚未初始化
  WORKING,        // 正在前往/处于工作点
  RETURNING_HOME  // 正在撤退/处于基地
};

class NavCommanderNode : public rclcpp::Node
{
public:
  NavCommanderNode() 
  : Node("nav_commander"), 
    current_state_(TacticalState::PENDING),
    is_system_ready_(false),
    has_received_hp_(false),
    current_hp_(100)
  {
    // 1. 初始化动作客户端、发布者、订阅者
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("nav_ready", 10);
    
    // 订阅 UDP 节点转发过来的血量
    hp_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/robot_hp", 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        this->hp_callback(msg);
      });

    // 2. 初始化 TF2 监听器 (用于检查底盘定位是否建立)
    // tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 3. 设置预定坐标点
    setup_poses();

    // 4. 开启一个 1Hz 的自检定时器
    init_check_timer_ = this->create_wall_timer(
      1s, [this]() { this->check_initialization(); });

    RCLCPP_INFO(this->get_logger(), "🔄 指挥节点已启动，正在执行系统级自检...");
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr hp_sub_;
  rclcpp::TimerBase::SharedPtr init_check_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  geometry_msgs::msg::PoseStamped work_goal_pose_;
  geometry_msgs::msg::PoseStamped home_pose_;
  
  TacticalState current_state_;
  bool is_system_ready_;
  bool has_received_hp_;
  int current_hp_;

  void setup_poses()
  {
    home_pose_.header.frame_id = "map";
    home_pose_.pose.position.x = 0.0;
    home_pose_.pose.position.y = 0.0;
    home_pose_.pose.orientation.w = 1.0;

    // work_goal_pose_.header.frame_id = "map";
    // work_goal_pose_.pose.position.x = 1.0;
    // work_goal_pose_.pose.position.y = 4.8;
    // work_goal_pose_.pose.orientation.w = 1.0;

    work_goal_pose_.header.frame_id = "map";
    work_goal_pose_.pose.position.x = 1.6;
    work_goal_pose_.pose.position.y = 3.6;
    work_goal_pose_.pose.orientation.w = 1.0;
  }

  // ==========================================
  // 步骤 1 & 2: 检测初始化 & 发送初始化信号
  // ==========================================
  void check_initialization()
  {
    if (is_system_ready_) return; // 如果已经就绪，直接跳过

    // if (!has_received_hp_) {
    //   RCLCPP_WARN(this->get_logger(), "自检中... 未收到电控血量数据");
    //   return;
    // }

    // 检查 2: Nav2 服务器是否启动
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "自检中... Nav2 服务器未上线");
      return;
    }

    // 检查 3: 空间定位是否完成 (map 到 base_link 的变换树是否存在)
    // std::string err_msg;
    // if (!tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero, &err_msg)) {
    //   RCLCPP_WARN(this->get_logger(), "自检中... 等待底盘定位 (TF: map -> base_link)");
    //   return;
    // }

    // --- 所有自检通过！---
    is_system_ready_ = true;
    init_check_timer_->cancel(); // 关闭自检定时器

    RCLCPP_INFO(this->get_logger(), "✅ 系统自检通过！");

    // 给电控发送初始化(就位)信号
    auto ready_msg = std_msgs::msg::Bool();
    ready_msg.data = true;
    ready_pub_->publish(ready_msg);

    // 触发第一波战术动作
    evaluate_tactics();
  }

  // ==========================================
  // 步骤 3: 接收血量
  // ==========================================
  void hp_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "收到血量更新: %d", msg->data);
    current_hp_ = msg->data;
    has_received_hp_ = true;

    // 只有在系统初始化完成后，血量变化才会触发导航指令
    if (is_system_ready_) {
      evaluate_tactics();
    }
  }

  // ==========================================
  // 步骤 4 & 5: 血量 > 50 前往工作点，否则回家
  // ==========================================
  void evaluate_tactics()
  {
    if (current_hp_ > 50) {
      RCLCPP_INFO(this->get_logger(), "🟢 状态更新 (HP: %d > 50) -> 下令：全速前往工作区！", current_hp_);
      current_state_ = TacticalState::WORKING;
      send_goal(work_goal_pose_);
    } 
    else if (current_hp_ <= 50 && current_state_ != TacticalState::RETURNING_HOME) {
      RCLCPP_WARN(this->get_logger(), "🔴 状态更新 (HP: %d <= 50) -> 下令：放弃任务，立即回城补血！", current_hp_);
      current_state_ = TacticalState::RETURNING_HOME;
      send_goal(home_pose_);
    }
  }

  // 发送底层导航指令
  void send_goal(const geometry_msgs::msg::PoseStamped& target_pose)
  {
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = target_pose;
    goal_msg.pose.header.stamp = this->now();

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
      [this](const GoalHandleNav::WrappedResult & result) {
        // 简单打印结果，不干扰血量驱动的状态机
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "🏁 导航已抵达当前战术目标点。");
        }
      };

    // 发送新目标，Nav2会自动取消之前的旧目标
    action_client_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavCommanderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}