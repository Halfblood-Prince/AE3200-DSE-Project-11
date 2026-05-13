#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/save_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using SaveMap = nav2_msgs::srv::SaveMap;
using SteadyClock = std::chrono::steady_clock;
using Cell = std::pair<int, int>;
using Point = std::pair<double, double>;

uint64_t cell_key(int x, int y)
{
    return (static_cast<uint64_t>(static_cast<uint32_t>(x)) << 32) |
           static_cast<uint32_t>(y);
}

class Nav2WaypointExplorer : public rclcpp::Node
{
  public:
    Nav2WaypointExplorer()
        : Node("nav2_waypoint_explorer"),
          tf_buffer_(get_clock()),
          tf_listener_(tf_buffer_)
    {
        declare_parameter<std::string>("map_topic", "/map_valid");
        declare_parameter<std::string>("map_save_path", "maps/complete_environment");
        declare_parameter<int>("min_exploration_goals", 10);
        declare_parameter<double>("frontier_timeout_sec", 45.0);
        declare_parameter<double>("initial_scan_sec", 10.0);
        declare_parameter<double>("loop_closure_settle_sec", 8.0);
        declare_parameter<double>("frontier_sample_step_m", 0.35);
        declare_parameter<double>("frontier_clearance_m", 0.45);
        declare_parameter<double>("frontier_min_distance_m", 3.0);
        declare_parameter<double>("frontier_max_distance_m", 18.0);
        declare_parameter<double>("frontier_unknown_radius_m", 0.9);
        declare_parameter<int>("frontier_min_unknown_cells", 6);
        declare_parameter<bool>("return_to_start", true);

        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        save_map_ = create_client<SaveMap>("map_saver/save_map");
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        const auto map_topic = get_parameter("map_topic").as_string();
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic,
            map_qos,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                map_ = std::move(msg);
            });
        timer_ = create_wall_timer(std::chrono::seconds(2), [this] {
            tick();
        });
    }

  private:
    enum class State
    {
        Exploring,
        Returning,
        Settling,
        Saving,
        Complete,
    };

    double parameter_double(const std::string &name) const
    {
        return get_parameter(name).as_double();
    }

    int parameter_int(const std::string &name) const
    {
        return static_cast<int>(get_parameter(name).as_int());
    }

    void tick()
    {
        if (state_ == State::Complete || active_)
        {
            return;
        }
        if (!map_)
        {
            const auto map_topic = get_parameter("map_topic").as_string();
            RCLCPP_INFO(
                get_logger(),
                "Waiting for %s before autonomous frontier exploration",
                map_topic.c_str());
            return;
        }
        if (!client_->wait_for_action_server(std::chrono::milliseconds(100)))
        {
            RCLCPP_INFO(get_logger(), "Waiting for Nav2 navigate_to_pose action server");
            return;
        }

        const auto robot = robot_pose();
        if (!robot)
        {
            return;
        }
        if (!start_pose_)
        {
            start_pose_ = robot;
            RCLCPP_INFO(
                get_logger(),
                "Recorded SLAM start pose at x=%.2f, y=%.2f",
                robot->first,
                robot->second);
        }

        switch (state_)
        {
        case State::Exploring:
            continue_exploration(*robot);
            break;
        case State::Returning:
            send_return_goal(*robot);
            break;
        case State::Settling:
            wait_then_save();
            break;
        case State::Saving:
            request_map_save();
            break;
        case State::Complete:
            break;
        }
    }

    void continue_exploration(const Point &robot)
    {
        const auto known_free = known_free_cell_count();
        const auto initial_scan_sec = parameter_double("initial_scan_sec");
        const auto elapsed_start = std::chrono::duration<double>(SteadyClock::now() - start_time_).count();
        if (goal_count_ == 0 && elapsed_start < initial_scan_sec)
        {
            publish_initial_scan_turn();
            RCLCPP_INFO(
                get_logger(),
                "Building the initial SLAM bubble before frontier navigation (%d free cells)",
                known_free);
            return;
        }

        const auto target = choose_frontier_goal(robot);
        if (target)
        {
            publish_stop();
            last_frontier_time_ = SteadyClock::now();
            send_goal(*target, robot, "frontier");
            return;
        }

        const auto elapsed = std::chrono::duration<double>(SteadyClock::now() - last_frontier_time_).count();
        const auto min_goals = parameter_int("min_exploration_goals");
        const auto timeout = parameter_double("frontier_timeout_sec");
        if (goal_count_ < min_goals || elapsed < timeout)
        {
            if (goal_count_ == 0 && known_free < 250)
            {
                publish_initial_scan_turn();
            }
            RCLCPP_INFO(
                get_logger(),
                "No reachable frontier goal yet; waiting for more known free space "
                "(%d free cells, %d/%d goals, %.0f/%.0fs quiet)",
                known_free,
                goal_count_,
                min_goals,
                elapsed,
                timeout);
            return;
        }

        publish_stop();
        if (get_parameter("return_to_start").as_bool())
        {
            RCLCPP_INFO(
                get_logger(),
                "No frontiers remain; returning near the start pose so slam_toolbox can close the loop");
            state_ = State::Returning;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "No frontiers remain; saving the completed map");
            state_ = State::Saving;
        }
    }

    void send_return_goal(const Point &robot)
    {
        if (!start_pose_)
        {
            state_ = State::Saving;
            return;
        }
        if (std::hypot(robot.first - start_pose_->first, robot.second - start_pose_->second) < 0.75)
        {
            RCLCPP_INFO(get_logger(), "Robot is back near the start pose; waiting for SLAM to settle");
            state_ = State::Settling;
            settle_started_at_ = SteadyClock::now();
            return;
        }
        send_goal(*start_pose_, robot, "loop-closure return");
    }

    void wait_then_save()
    {
        const auto settle = parameter_double("loop_closure_settle_sec");
        if (!settle_started_at_)
        {
            settle_started_at_ = SteadyClock::now();
        }
        const auto elapsed = std::chrono::duration<double>(SteadyClock::now() - *settle_started_at_).count();
        if (elapsed < settle)
        {
            return;
        }
        RCLCPP_INFO(get_logger(), "SLAM settle period complete; saving map and leaving Nav2 ready");
        state_ = State::Saving;
    }

    void request_map_save()
    {
        if (save_requested_)
        {
            return;
        }
        if (!save_map_->wait_for_service(std::chrono::milliseconds(100)))
        {
            RCLCPP_INFO(get_logger(), "Waiting for map_saver/save_map service");
            return;
        }

        const auto map_url = get_parameter("map_save_path").as_string();
        const auto parent = std::filesystem::path(map_url).parent_path();
        if (!parent.empty())
        {
            std::filesystem::create_directories(parent);
        }

        auto request = std::make_shared<SaveMap::Request>();
        request->map_topic = get_parameter("map_topic").as_string();
        request->map_url = map_url;
        request->image_format = "pgm";
        request->map_mode = "trinary";
        request->free_thresh = 0.25;
        request->occupied_thresh = 0.65;

        save_requested_ = true;
        save_map_->async_send_request(
            request,
            [this](rclcpp::Client<SaveMap>::SharedFuture future) {
                map_saved(future);
            });
    }

    void map_saved(const rclcpp::Client<SaveMap>::SharedFuture &future)
    {
        try
        {
            const auto response = future.get();
            if (response->result)
            {
                const auto path = get_parameter("map_save_path").as_string();
                RCLCPP_INFO(
                    get_logger(),
                    "Saved completed map to %s.yaml/.pgm. Exploration is stopped; "
                    "Nav2 remains active for pathfinding goals.",
                    path.c_str());
                state_ = State::Complete;
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "map_saver reported failure; will retry");
                save_requested_ = false;
            }
        }
        catch (const std::exception &error)
        {
            RCLCPP_ERROR(get_logger(), "Map save failed: %s", error.what());
            save_requested_ = false;
        }
    }

    std::optional<Point> robot_pose()
    {
        try
        {
            const auto transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            return Point{
                transform.transform.translation.x,
                transform.transform.translation.y,
            };
        }
        catch (const tf2::TransformException &error)
        {
            RCLCPP_INFO(get_logger(), "Waiting for map -> base_link TF: %s", error.what());
            return std::nullopt;
        }
    }

    int known_free_cell_count() const
    {
        if (!map_)
        {
            return 0;
        }
        return static_cast<int>(std::count_if(map_->data.begin(), map_->data.end(), [](int8_t value) {
            return value >= 0 && value < 50;
        }));
    }

    void publish_initial_scan_turn()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = 0.28;
        cmd_pub_->publish(cmd);
    }

    void publish_stop()
    {
        cmd_pub_->publish(geometry_msgs::msg::Twist{});
    }

    std::optional<Point> choose_frontier_goal(const Point &robot)
    {
        const auto robot_cell = world_to_cell(robot.first, robot.second);
        if (!robot_cell)
        {
            return std::nullopt;
        }

        safe_cell_cache_.clear();
        safe_cell_cache_enabled_ = true;
        auto reachable = reachable_safe_cells(*robot_cell);
        safe_cell_cache_enabled_ = false;
        safe_cell_cache_.clear();

        if (reachable.empty())
        {
            RCLCPP_WARN(get_logger(), "No reachable known-free cells found around the robot yet");
            return std::nullopt;
        }

        const auto resolution = map_->info.resolution;
        const auto step = std::max(1, static_cast<int>(parameter_double("frontier_sample_step_m") / resolution));
        const auto min_distance = parameter_double("frontier_min_distance_m");
        const auto max_distance = parameter_double("frontier_max_distance_m");
        const auto unknown_radius = parameter_double("frontier_unknown_radius_m");
        const auto min_unknown = parameter_int("frontier_min_unknown_cells");

        std::optional<Point> best;
        auto best_score = -std::numeric_limits<double>::infinity();
        size_t sampled = 0;
        size_t near_unknown = 0;

        for (const auto &cell : reachable)
        {
            const auto mx = cell.first;
            const auto my = cell.second;
            if (mx % step || my % step)
            {
                continue;
            }
            ++sampled;

            const auto world = cell_to_world(mx, my);
            const auto robot_distance = std::hypot(world.first - robot.first, world.second - robot.second);
            if (robot_distance < min_distance || robot_distance > max_distance)
            {
                continue;
            }
            if (recently_seen(world.first, world.second))
            {
                continue;
            }

            const auto unknown = unknown_neighbor_count(mx, my, unknown_radius);
            if (unknown < min_unknown)
            {
                continue;
            }
            ++near_unknown;

            auto start_bonus = 0.0;
            if (start_pose_)
            {
                start_bonus = 0.1 * std::hypot(world.first - start_pose_->first, world.second - start_pose_->second);
            }
            const auto distance_bonus = std::min(robot_distance, max_distance) * 0.8;
            const auto score = unknown + distance_bonus + start_bonus;
            if (score > best_score)
            {
                best = world;
                best_score = score;
            }
        }

        if (!best)
        {
            RCLCPP_INFO(
                get_logger(),
                "Frontier scan found no target (%zu reachable cells, %zu sampled, %zu near unknown)",
                reachable.size(),
                sampled,
                near_unknown);
        }
        return best;
    }

    Point cell_to_world(int mx, int my) const
    {
        const auto &origin = map_->info.origin.position;
        return Point{
            origin.x + (mx + 0.5) * map_->info.resolution,
            origin.y + (my + 0.5) * map_->info.resolution,
        };
    }

    std::optional<Cell> world_to_cell(double x, double y) const
    {
        const auto &origin = map_->info.origin.position;
        const auto mx = static_cast<int>((x - origin.x) / map_->info.resolution);
        const auto my = static_cast<int>((y - origin.y) / map_->info.resolution);
        if (mx >= 0 && my >= 0 && mx < static_cast<int>(map_->info.width) &&
            my < static_cast<int>(map_->info.height))
        {
            return Cell{mx, my};
        }
        return std::nullopt;
    }

    int8_t cell_value(int mx, int my) const
    {
        return map_->data[static_cast<size_t>(my) * map_->info.width + static_cast<size_t>(mx)];
    }

    bool is_safe_free_cell(int mx, int my)
    {
        if (mx < 0 || my < 0 || mx >= static_cast<int>(map_->info.width) ||
            my >= static_cast<int>(map_->info.height))
        {
            return false;
        }

        const auto key = cell_key(mx, my);
        if (safe_cell_cache_enabled_)
        {
            const auto match = safe_cell_cache_.find(key);
            if (match != safe_cell_cache_.end())
            {
                return match->second;
            }
        }

        const auto center_value = cell_value(mx, my);
        if (center_value < 0 || center_value >= 50)
        {
            return cache_safe_cell(key, false);
        }

        const auto clearance = parameter_double("frontier_clearance_m");
        const auto clearance_cells = std::max(2, static_cast<int>(clearance / map_->info.resolution));
        auto safe = true;
        for (int dy = -clearance_cells; dy <= clearance_cells && safe; ++dy)
        {
            for (int dx = -clearance_cells; dx <= clearance_cells; ++dx)
            {
                if (dx * dx + dy * dy > clearance_cells * clearance_cells)
                {
                    continue;
                }
                const auto x = mx + dx;
                const auto y = my + dy;
                if (x < 0 || y < 0 || x >= static_cast<int>(map_->info.width) ||
                    y >= static_cast<int>(map_->info.height))
                {
                    safe = false;
                    break;
                }
                if (cell_value(x, y) >= 50)
                {
                    safe = false;
                    break;
                }
            }
        }

        return cache_safe_cell(key, safe);
    }

    bool cache_safe_cell(uint64_t key, bool value)
    {
        if (safe_cell_cache_enabled_)
        {
            safe_cell_cache_[key] = value;
        }
        return value;
    }

    std::vector<Cell> reachable_safe_cells(Cell start)
    {
        if (!is_safe_free_cell(start.first, start.second))
        {
            const auto nearby = nearest_safe_cell(start);
            if (!nearby)
            {
                return {};
            }
            start = *nearby;
        }

        std::deque<Cell> queue{start};
        std::unordered_set<uint64_t> visited{cell_key(start.first, start.second)};
        std::vector<Cell> cells;

        while (!queue.empty())
        {
            const auto [mx, my] = queue.front();
            queue.pop_front();
            cells.emplace_back(mx, my);

            const Cell neighbors[] = {
                {mx + 1, my},
                {mx - 1, my},
                {mx, my + 1},
                {mx, my - 1},
            };
            for (const auto &neighbor : neighbors)
            {
                const auto key = cell_key(neighbor.first, neighbor.second);
                if (visited.count(key))
                {
                    continue;
                }
                if (!is_safe_free_cell(neighbor.first, neighbor.second))
                {
                    continue;
                }
                visited.insert(key);
                queue.push_back(neighbor);
            }
        }

        return cells;
    }

    std::optional<Cell> nearest_safe_cell(Cell start)
    {
        const auto max_radius = std::max(2, static_cast<int>(1.0 / map_->info.resolution));
        for (int radius = 1; radius <= max_radius; ++radius)
        {
            for (int dy = -radius; dy <= radius; ++dy)
            {
                for (int dx = -radius; dx <= radius; ++dx)
                {
                    if (std::abs(dx) != radius && std::abs(dy) != radius)
                    {
                        continue;
                    }
                    const auto mx = start.first + dx;
                    const auto my = start.second + dy;
                    if (mx >= 0 && my >= 0 && mx < static_cast<int>(map_->info.width) &&
                        my < static_cast<int>(map_->info.height) && is_safe_free_cell(mx, my))
                    {
                        return Cell{mx, my};
                    }
                }
            }
        }
        return std::nullopt;
    }

    int unknown_neighbor_count(int mx, int my, double radius_m) const
    {
        const auto radius = std::max(2, static_cast<int>(radius_m / map_->info.resolution));
        auto count = 0;
        for (int dy = -radius; dy <= radius; ++dy)
        {
            for (int dx = -radius; dx <= radius; ++dx)
            {
                const auto x = mx + dx;
                const auto y = my + dy;
                if (x < 0 || y < 0 || x >= static_cast<int>(map_->info.width) ||
                    y >= static_cast<int>(map_->info.height))
                {
                    continue;
                }
                if (cell_value(x, y) < 0)
                {
                    ++count;
                }
            }
        }
        return count;
    }

    bool recently_seen(double x, double y) const
    {
        const auto check = [x, y](const Point &point) {
            return std::hypot(x - point.first, y - point.second) < 0.9;
        };

        const auto failed_start = failed_goals_.size() > 30 ? failed_goals_.size() - 30 : 0;
        for (size_t index = failed_start; index < failed_goals_.size(); ++index)
        {
            if (check(failed_goals_[index]))
            {
                return true;
            }
        }

        const auto visited_start = visited_goals_.size() > 50 ? visited_goals_.size() - 50 : 0;
        for (size_t index = visited_start; index < visited_goals_.size(); ++index)
        {
            if (check(visited_goals_[index]))
            {
                return true;
            }
        }
        return false;
    }

    void send_goal(const Point &target, const Point &robot, const char *label)
    {
        const auto yaw = std::atan2(target.second - robot.second, target.first - robot.first);
        NavigateToPose::Goal goal;
        goal.pose = make_pose(target.first, target.second, yaw);

        ++goal_count_;
        RCLCPP_INFO(
            get_logger(),
            "Sending Nav2 %s goal %d: x=%.2f, y=%.2f, yaw=%.2f",
            label,
            goal_count_,
            target.first,
            target.second,
            yaw);

        active_ = true;
        current_goal_ = target;

        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
        options.goal_response_callback = [this](const GoalHandleNavigate::SharedPtr &goal_handle) {
            goal_response_callback(goal_handle);
        };
        options.result_callback = [this](const GoalHandleNavigate::WrappedResult &result) {
            result_callback(result);
        };
        client_->async_send_goal(goal, options);
    }

    void goal_response_callback(const GoalHandleNavigate::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_WARN(get_logger(), "Nav2 goal rejected; choosing a new goal");
            if (current_goal_)
            {
                failed_goals_.push_back(*current_goal_);
            }
            current_goal_.reset();
            active_ = false;
        }
    }

    void result_callback(const GoalHandleNavigate::WrappedResult &result)
    {
        RCLCPP_INFO(get_logger(), "Nav2 goal finished with status %d", static_cast<int>(result.code));
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            if (current_goal_)
            {
                visited_goals_.push_back(*current_goal_);
            }
        }
        else if (current_goal_)
        {
            failed_goals_.push_back(*current_goal_);
        }

        if (state_ == State::Returning)
        {
            state_ = State::Settling;
            settle_started_at_ = SteadyClock::now();
        }
        current_goal_.reset();
        active_ = false;
    }

    geometry_msgs::msg::PoseStamped make_pose(double x, double y, double yaw)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = get_clock()->now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.z = std::sin(yaw / 2.0);
        pose.pose.orientation.w = std::cos(yaw / 2.0);
        return pose;
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::Client<SaveMap>::SharedPtr save_map_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    State state_{State::Exploring};
    bool active_{false};
    int goal_count_{0};
    std::optional<Point> current_goal_;
    std::vector<Point> failed_goals_;
    std::vector<Point> visited_goals_;
    std::optional<Point> start_pose_;
    SteadyClock::time_point start_time_{SteadyClock::now()};
    SteadyClock::time_point last_frontier_time_{SteadyClock::now()};
    std::optional<SteadyClock::time_point> settle_started_at_;
    bool save_requested_{false};
    bool safe_cell_cache_enabled_{false};
    std::unordered_map<uint64_t, bool> safe_cell_cache_;
};
} // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nav2WaypointExplorer>());
    rclcpp::shutdown();
    return 0;
}
