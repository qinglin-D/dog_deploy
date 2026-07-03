#include "gamepad/gamepad.h"

namespace GAMEPAD
{

Gamepad::Gamepad(GamepadMapping mapping)
    : mapping_(std::move(mapping))
{
    axes_bound_ = std::max({mapping_.axis_x, mapping_.axis_y, mapping_.axis_yaw, mapping_.axis_trigger}) + 1;
    buttons_bound_ = std::max({mapping_.button_terminate, mapping_.button_pause,
                               mapping_.button_reset, mapping_.button_go_on,
                               mapping_.button_policy_1, mapping_.button_policy_2,
                               mapping_.button_policy_3}) + 1;
}

void Gamepad::make_subscription(rclcpp::Node::SharedPtr node)
{
    subscriber_ = node->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&Gamepad::callback, this, std::placeholders::_1));
    publisher_ = node->create_publisher<std_msgs::msg::Int32>("button_state", 10);
    policy_index_publisher_ = node->create_publisher<std_msgs::msg::Int32>("policy_index", 10);
    timer_ = node->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Gamepad::publish, this));
}

void Gamepad::callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (!connected_.load(std::memory_order_acquire)) {
        size_t buttons_size = msg->buttons.size();
        size_t axes_size = msg->axes.size();
        if (static_cast<size_t>(axes_bound_) > axes_size || static_cast<size_t>(buttons_bound_) > buttons_size) {
            fprintf(stdout, "Received Joy message with insufficient axes/buttons: %zu axes, %zu buttons", axes_size, buttons_size);
            return;
        }
        connected_.store(true, std::memory_order_release);
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        axes_[0] = msg->axes[mapping_.axis_x];
        axes_[1] = msg->axes[mapping_.axis_y];
        axes_[2] = msg->axes[mapping_.axis_yaw];
        axes_[3] = msg->axes[mapping_.axis_trigger];
    }

    if (msg->buttons[mapping_.button_pause] > 0) {
        states_.store(Gamepad::PAUSE, std::memory_order_release);
    } else if (msg->buttons[mapping_.button_go_on] > 0) {
        states_.store(Gamepad::GO_ON, std::memory_order_release);
    } else if (msg->buttons[mapping_.button_reset] > 0) {
        states_.store(Gamepad::RESET, std::memory_order_release);
    } else if (msg->buttons[mapping_.button_terminate] > 0) {
        states_.store(Gamepad::TERMINATE, std::memory_order_release);
    }

    if (msg->buttons[mapping_.button_policy_1] > 0) {
        policy_index_.store(0, std::memory_order_release);
    } else if (msg->buttons[mapping_.button_policy_2] > 0) {
        policy_index_.store(1, std::memory_order_release);
    } else if (msg->buttons[mapping_.button_policy_3] > 0) {
        policy_index_.store(2, std::memory_order_release);
    }
}

void Gamepad::publish()
{
    auto state_msg = std_msgs::msg::Int32();
    state_msg.data = static_cast<int>(states_.load(std::memory_order_acquire));
    publisher_->publish(state_msg);

    auto index_msg = std_msgs::msg::Int32();
    index_msg.data = policy_index_.load(std::memory_order_acquire);
    policy_index_publisher_->publish(index_msg);
}

void Gamepad::get_axes(double* axes) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    axes[0] = axes_[0];
    axes[1] = axes_[1];
    axes[2] = axes_[2];
    axes[3] = axes_[3];
}

} // namespace GAMEPAD
