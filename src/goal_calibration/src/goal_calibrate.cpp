#include <memory>
#include <queue>
#include <set>
#include <cmath>
#include <vector>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

using std::placeholders::_1;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class GoalFilterNode : public rclcpp::Node
{
public:
    GoalFilterNode() : Node("goal_calibrate")
    {
        // ==========================================
        // 物理碰撞与搜索参数
        // ==========================================
        safe_center_cost_ = 0;
        clearance_radius_m_ = 0.10;
        max_search_radius_m_ = 3.0;

        auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/red_standard_robot1/global_costmap/costmap",
            costmap_qos,
            std::bind(&GoalFilterNode::costmap_callback, this, _1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/red_standard_robot1/goal_pose_raw",
            10,
            std::bind(&GoalFilterNode::goal_callback, this, _1));

        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "/red_standard_robot1/navigate_to_pose");

        RCLCPP_INFO(this->get_logger(), "r: %.2fm", clearance_radius_m_);
    }

private:
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

    int safe_center_cost_;
    double clearance_radius_m_;
    double max_search_radius_m_;

    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        costmap_ = msg;
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!costmap_) {
            RCLCPP_WARN(this->get_logger(), "No costmap received yet. Cannot process goal.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received goal_pose_raw: (%.2f, %.2f)", msg->pose.position.x, msg->pose.position.y);

        auto safe_pose_opt = find_nearest_safe_pose(*msg);

        if (safe_pose_opt.has_value()) {
            auto safe_pose = safe_pose_opt.value();
            RCLCPP_INFO(this->get_logger(), "safe pose: (%.2f, %.2f)", 
                        safe_pose.pose.position.x, safe_pose.pose.position.y);
            send_nav_goal(safe_pose);
        } else {
            RCLCPP_ERROR(this->get_logger(), "No available safe pose found within search radius. Goal rejected.");
        }
    }

    bool is_area_absolutely_clear(int center_mx, int center_my, int width, int height, double resolution)
    {
        int check_cells = static_cast<int>(clearance_radius_m_ / resolution);

        for (int dx = -check_cells; dx <= check_cells; ++dx) {
            for (int dy = -check_cells; dy <= check_cells; ++dy) {
                if (dx * dx + dy * dy > check_cells * check_cells) {
                    continue;
                }

                int check_mx = center_mx + dx;
                int check_my = center_my + dy;

                if (check_mx < 0 || check_mx >= width || check_my < 0 || check_my >= height) {
                    return false;
                }

                int idx = check_my * width + check_mx;
                int8_t cost = costmap_->data[idx];

                if (cost > 0 || cost == -1) {
                    return false;
                }
            }
        }
        return true;
    }

    std::optional<geometry_msgs::msg::PoseStamped> find_nearest_safe_pose(const geometry_msgs::msg::PoseStamped& goal_pose)
    {
        double resolution = costmap_->info.resolution;
        double origin_x = costmap_->info.origin.position.x;
        double origin_y = costmap_->info.origin.position.y;
        int width = costmap_->info.width;
        int height = costmap_->info.height;

        double world_x = goal_pose.pose.position.x;
        double world_y = goal_pose.pose.position.y;

        int start_mx = std::floor((world_x - origin_x) / resolution);
        int start_my = std::floor((world_y - origin_y) / resolution);

        if (start_mx < 0 || start_mx >= width || start_my < 0 || start_my >= height) {
            RCLCPP_WARN(this->get_logger(), "out of map bounds: (%d, %d)", start_mx, start_my);
            return std::nullopt;
        }

        int start_index = start_my * width + start_mx;
        int8_t start_cost = costmap_->data[start_index];
        

        if (start_cost >= 253 || start_cost == -1) {
            RCLCPP_WARN(this->get_logger(), "...");
        } else {
            if (is_area_absolutely_clear(start_mx, start_my, width, height, resolution)) {
                RCLCPP_INFO(this->get_logger(), "ok.");
                return goal_pose;
            } else {
                RCLCPP_WARN(this->get_logger(), "too close to obstacle");
            }
        }

        // BFS 初始化
        int max_search_cells = static_cast<int>(max_search_radius_m_ / resolution);
        std::queue<std::pair<int, int>> q;
        std::set<std::pair<int, int>> visited;
        
        std::vector<std::pair<int, int>> directions = {
            {0, 1}, {0, -1}, {1, 0}, {-1, 0},
            {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
        };

        q.push({start_mx, start_my});
        visited.insert({start_mx, start_my});

        while (!q.empty()) {
            auto [curr_mx, curr_my] = q.front();
            q.pop();

            if (std::abs(curr_mx - start_mx) > max_search_cells || std::abs(curr_my - start_my) > max_search_cells) {
                continue;
            }

            int curr_index = curr_my * width + curr_mx;
            int8_t cost = costmap_->data[curr_index];

            if (cost == safe_center_cost_ && cost != -1) {
                if (is_area_absolutely_clear(curr_mx, curr_my, width, height, resolution)) {
                    double safe_world_x = (curr_mx + 0.5) * resolution + origin_x;
                    double safe_world_y = (curr_my + 0.5) * resolution + origin_y;

                    geometry_msgs::msg::PoseStamped new_pose;
                    new_pose.header = goal_pose.header;
                    new_pose.pose.position.x = safe_world_x;
                    new_pose.pose.position.y = safe_world_y;
                    new_pose.pose.orientation = goal_pose.pose.orientation;
                    return new_pose;
                }
            }

            for (const auto& dir : directions) {
                int next_mx = curr_mx + dir.first;
                int next_my = curr_my + dir.second;

                if (next_mx >= 0 && next_mx < width && next_my >= 0 && next_my < height) {
                    if (visited.find({next_mx, next_my}) == visited.end()) {
                        visited.insert({next_mx, next_my});
                        q.push({next_mx, next_my});
                    }
                }
            }
        }

        return std::nullopt;
    }

    void send_nav_goal(const geometry_msgs::msg::PoseStamped& pose)
    {
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 Action server not available after waiting. Goal not sent.");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = pose;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}