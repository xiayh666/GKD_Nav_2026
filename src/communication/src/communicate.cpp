#include "communicate.hpp"
#include "protocal.h"
#include "socket_server.hpp"
#include <cstdlib>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/detail/int32__struct.hpp>

bool is_dead = false;

std::atomic<bool> CmdVelSubscriber::stop_requested_ = false;
std::atomic<bool> CmdVelSubscriber::auto_aim_captured_ = false;
std::atomic<int> CmdVelSubscriber::game_progress_ = false;
std::atomic<bool> CmdVelSubscriber::startup_finished_ = false;
std::atomic<bool> CmdVelSubscriber::startup_timer_created_ = false;



std::shared_ptr<CmdVelSubscriber>
    CmdVelSubscriber::cmd_vel_subscriber_instance_ = nullptr;

CmdVelSubscriber::CmdVelSubscriber() : Node("cmd_vel_subscriber") {

  hp_publisher_ =
      rclcpp::Node::create_publisher<std_msgs::msg::Int32>("robot_hp", 10);

  ser_ = std::make_shared<SocketServer<ReceiveNavigationInfo>>(
      11456, [this](const ReceiveNavigationInfo &info) {
        // RCLCPP_INFO(
        //     rclcpp::get_logger("socket_server"),
        //     "Received from UDP: yaw=%.2f, hp=%.2f, start=%d,
        //     auto_aim_captured=%d", info.yaw, info.hp, info.start,
        //     info.auto_aim_captured);

        auto_aim_captured_ = info.auto_aim_captured;
        int hp = info.hp;
        game_progress_ = info.game_progress;

        auto hp_msg = std_msgs::msg::Int32();

        hp_msg.data = int(hp);

        if (hp_publisher_) {
          hp_publisher_->publish(hp_msg);
        }

        if (hp <= 0) {
          RCLCPP_INFO(this->get_logger(), "hp <= 0, is_dead = true");
          is_dead = true;
        } else if (is_dead) {

          is_dead = false;
          respawn_init_ = true;
          RCLCPP_INFO(this->get_logger(), "reset is_dead to false");
          respawn_timer_ = this->create_wall_timer(10s, [this]() {
            if (respawn_timer_) {
              respawn_timer_->cancel();
            }

            respawn_init_ = false;
            RCLCPP_INFO(this->get_logger(), "reset respawn_init_ to false");
          });
        }
      });

  subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/red_standard_robot1/cmd_vel", rclcpp::QoS(1),
      std::bind(&CmdVelSubscriber::listener_callback, this,
                std::placeholders::_1));

  subscription_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/red_standard_robot1/map", rclcpp::QoS(1),
      std::bind(&CmdVelSubscriber::check_map_callback, this,
                std::placeholders::_1));

  check_zero_timer_ = this->create_wall_timer(
      100ms, std::bind(&CmdVelSubscriber::check_zero, this));

  latest_msg_ = nullptr;
  latest_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME); // 无效时间

  send_control_thread_ =
      std::thread(&CmdVelSubscriber::send_control_task, this);
}

CmdVelSubscriber::~CmdVelSubscriber() {
  if (send_control_thread_.joinable()) {
    send_control_thread_.join();
  }
}

void CmdVelSubscriber::init_finished() { initialised_ = true; }

std::shared_ptr<CmdVelSubscriber> CmdVelSubscriber::get_instance() {
  if (!CmdVelSubscriber::cmd_vel_subscriber_instance_) {
    auto instance = std::make_shared<CmdVelSubscriber>();
    cmd_vel_subscriber_instance_ = instance;
    return instance;
  } else {
    return CmdVelSubscriber::cmd_vel_subscriber_instance_;
  }
}

void CmdVelSubscriber::handle_sigint(int signum) {
  if (signum == SIGINT) {
    if (!cmd_vel_subscriber_instance_) {
      std::cout << "received sigint but cmd_vel_subscriver is nullptr"
                << std::endl;
    } else {
      cmd_vel_subscriber_instance_->stop_requested_ = true;
      send_control(0, 0);
      std::cout << "received sigint, terminating..." << std::endl;
    }
  }
}

void CmdVelSubscriber::listener_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_time_ = this->now();
  latest_msg_ = std::make_shared<geometry_msgs::msg::Twist>(*msg);
}

void CmdVelSubscriber::check_map_callback(
    const nav_msgs::msg::OccupancyGrid msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!msg.data.empty()) {
    // RCLCPP_INFO(this->get_logger(), "map msg received");
    init_finished();
  } else {
    RCLCPP_INFO(this->get_logger(), "no map msg");
  }
}

void CmdVelSubscriber::check_zero() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (latest_time_.nanoseconds() == 0) {
    return;
  }
  rclcpp::Duration diff = this->now() - latest_time_;
  if (diff.seconds() > 0.3) {
    latest_msg_.reset();
  }
}

void CmdVelSubscriber::send_control(double x, double y) {
  // pkg.initialised = initialised;
  if (game_progress_ != 4 && false) {
    SendNavigationInfo pkg;
    pkg.vx = pkg.vy = 0;
    pkg.enable_rotate = false;
    pkg.robot_mode = ROBOT_FOLLOW_GIMBAL;
    pkg.enable_auto_aim = false;

    cmd_vel_subscriber_instance_->ser_->send<SendNavigationInfo>(pkg);
    return;
  }
  if (!startup_finished_) {
    if (!startup_timer_created_) {
      if (!cmd_vel_subscriber_instance_) {
        std::cout << "cmd_vel_subscriver_instane_ = nullptr" << std::endl;
      } else {
        cmd_vel_subscriber_instance_->startup();
      }
    } else {
      std::cout << "startup msg sending..." << std::endl;
      SendNavigationInfo pkg;
      pkg.vx = 0;
      pkg.vy = 0.5;
      pkg.header = 0x37;
      pkg.enable_rotate = false;
      pkg.robot_mode = ROBOT_FOLLOW_GIMBAL;
      pkg.enable_auto_aim = false;

      cmd_vel_subscriber_instance_->ser_->send<SendNavigationInfo>(pkg);
    }
    return;
  }

  if (!cmd_vel_subscriber_instance_) {
    std::cout << "cmd_vel_subscriver_instane_ = nullptr" << std::endl;
  } else {
    SendNavigationInfo pkg{};
    pkg.header = 0x37;
    pkg.vx = x;
    pkg.vy = y;
    if ((CmdVelSubscriber::get_instance()->initialised_) &&
        !get_instance()->respawn_init_) {
      pkg.enable_rotate = 1;
      pkg.robot_mode = CmdVelSubscriber::auto_aim_captured_
                           ? ROBOT_FOLLOW_GIMBAL
                           : ROBOT_SEARCH;
    } else {
      pkg.enable_rotate = 0;
      pkg.robot_mode = ROBOT_FOLLOW_GIMBAL;
    }

    // pkg.enable_rotate = 0;

    // pkg.robot_mode = ROBOT_FOLLOW_GIMBAL;

    // pkg.robot_mode = ROBOT_SEARCH;

    pkg.enable_auto_aim = cmd_vel_subscriber_instance_->auto_aim_captured_;

    // std::cout << (pkg.robot_mode == ROBOT_FOLLOW_GIMBAL ? "FOLLOW_GIMBAL"
    //                                                     : "SEARCH")
    //           << std::endl
    //           << (pkg.enable_auto_aim ? "ENABLE_AUTOAIM" : "DISABLE_AUTOAIM")
    //           << std::endl;
    cmd_vel_subscriber_instance_->ser_->send<SendNavigationInfo>(pkg);
  }
}

void CmdVelSubscriber::send_control_task() {
  while (!stop_requested_ && rclcpp::ok()) {
    auto start_time = std::chrono::high_resolution_clock::now();
    std::shared_ptr<geometry_msgs::msg::Twist> local_msg;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      local_msg = latest_msg_;
    }

    if (!local_msg) {
      send_control(0, 0);
    } else {
      send_control(local_msg->linear.y, local_msg->linear.x);
      // send_control(0, 0);

      std::cout << "cmd_vel: "
                << "(" << local_msg->linear.y << ", " << local_msg->linear.x
                << ", "
                << ")" << std::endl;
    }

    std::this_thread::sleep_for(
        1ms - (std::chrono::high_resolution_clock::now() - start_time));
  }
  std::cout << "end" << std::endl;
  rclcpp::shutdown();
}

void CmdVelSubscriber::startup() {
  startup_timer_ = this->create_wall_timer(1s, [this]() {
    startup_finished_ = true;
    startup_timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "startup finished, startup_timer cancelled");
  });
  startup_timer_created_ = true;
}

int main() {
  rclcpp::init(0, nullptr);
  signal(SIGINT, &CmdVelSubscriber::handle_sigint);
  auto cmd_vel_subscriber = CmdVelSubscriber::get_instance();

  rclcpp::spin(cmd_vel_subscriber);
  rclcpp::shutdown();

  return 0;
}
