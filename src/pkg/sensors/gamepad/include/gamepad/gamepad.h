#pragma once

#include <array>
#include <atomic>
#include <mutex>

#include "sensor_msgs/msg/joy.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

namespace GAMEPAD
{

struct GamepadMapping
{
    int axis_x{1};
    int axis_y{0};
    int axis_yaw{4};
    int axis_trigger{3};

    int button_terminate{0};
    int button_pause{1};
    int button_go_on{2};
    int button_reset{3};
    int button_policy_1{4};
    int button_policy_2{5};
    int button_policy_3{6};
};


class Gamepad
{
public:
    typedef enum {
        PAUSE = 0,
        GO_ON = 1,
        TERMINATE = 2,
        RESET = 3
    }States;

    Gamepad(GamepadMapping mapping = {});

    void make_subscription(rclcpp::Node::SharedPtr node);

    void get_axes(double* axes) const;
    int get_policy_index(){
        return policy_index_.load(std::memory_order_acquire);
    }
    States get_states(){
        return states_.load(std::memory_order_acquire);
    }

    bool is_connected() const { return connected_.load(std::memory_order_acquire); }

private:
    void callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void publish();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr policy_index_publisher_;

    mutable std::mutex mutex_;
    std::array<double, 4> axes_{};
    std::atomic<States> states_{GO_ON};
    std::atomic<int> policy_index_{0};
    std::atomic<bool> connected_{false};

    int axes_bound_;
    int buttons_bound_;
    GamepadMapping mapping_;
};

} // namespace GAMEPAD
