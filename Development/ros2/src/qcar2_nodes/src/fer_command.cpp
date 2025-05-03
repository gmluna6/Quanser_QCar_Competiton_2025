#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include "std_msgs/msg/header.hpp"

#include "quanser/quanser_messages.h"
#include "quanser/quanser_memory.h"
#include "quanser/quanser_hid.h"

#include "qcar2_interfaces/msg/motor_commands.hpp"
#include "qcar2_interfaces/msg/boolean_leds.hpp"

using namespace std::chrono_literals;

class CommandPublisher : public rclcpp::Node
{
public:
    CommandPublisher()
    : Node("joystick_publisher")
    {
        // Publishers
        command_publisher_ = this->create_publisher<qcar2_interfaces::msg::MotorCommands>("qcar2_motor_speed_cmd", 10);
        led_publisher_ = this->create_publisher<qcar2_interfaces::msg::BooleanLeds>("qcar2_led_cmd", 10);

        // Controller settings
        controller_number = 1;  // because js0
        result = game_controller_open(
            controller_number,
            buffer_size,
            deadzone,
            saturation,
            auto_center,
            max_force_feedback_effects,
            force_feedback_gain,
            &gamepad
        );

        if (result < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open game controller. Error code: %d", result);
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Game controller opened successfully.");
        }

        // Timer loop to poll joystick
        timer_ = this->create_wall_timer(50ms, std::bind(&CommandPublisher::timer_callback, this));
    }

    ~CommandPublisher()
    {
        game_controller_close(gamepad);
    }

private:
    void timer_callback()
    {
        result = game_controller_poll(gamepad, &data, &is_new);
        if (result < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to poll controller. Error code: %d", result);
            return;
        }

        if (!is_new)
            return;

        // === Extract inputs ===
        double LLA = -1 * data.x;   // left joystick horizontal (steering)
        double RT = data.rz;        // right trigger (throttle)
        bool A = data.buttons & (1 << 0);    // A button
        bool LB = data.buttons & (1 << 4);   // Left bumper

        double throttle = 0.0;
        double steering = 0.0;

        bool led_values[16] = {false};

        if (LB == 1)
        {
            // Front and rear lights ON
            led_values[8] = led_values[9] = led_values[10] = led_values[11] = true;
            led_values[12] = led_values[13] = true;

            throttle = RT == 0 ? 0 : 0.3 * (0.5 + 0.5 * RT);
            steering = 0.5 * LLA;

            if (steering > 0.01) {
                led_values[14] = true;
                led_values[6] = true;
            } else if (steering < -0.01) {
                led_values[15] = true;
                led_values[7] = true;
            }

            if (A == 1) {
                throttle = -throttle;
                led_values[4] = led_values[5] = true;
            }
        }
        else
        {
            // Brake lights on, no motion
            led_values[0] = led_values[1] = led_values[2] = led_values[3] = true;
            throttle = 0;
            steering = 0;
        }

        // === Publish Motor Command ===
        qcar2_interfaces::msg::MotorCommands motor_msg;
        motor_msg.motor_names = {"steering_angle", "motor_throttle"};
        motor_msg.values = {steering, throttle};
        command_publisher_->publish(motor_msg);

        // === Publish LED Command ===
        qcar2_interfaces::msg::BooleanLeds led_msg;
        led_msg.led_names = {
            "left_outside_brake_light",
            "left_inside_brake_light",
            "right_inside_brake_light",
            "right_outside_brake_light",
            "left_reverse_light",
            "right_reverse_light",
            "left_rear_signal",
            "right_rear_signal",
            "left_outside_headlight",
            "left_middle_headlight",
            "left_inside_headlight",
            "right_inside_headlight",
            "right_middle_headlight",
            "right_outside_headlight",
            "left_front_signal",
            "right_front_signal"
        };
        led_msg.values = std::vector<bool>(std::begin(led_values), std::end(led_values));
        led_publisher_->publish(led_msg);
    }

    // Publishers and timer
    rclcpp::Publisher<qcar2_interfaces::msg::MotorCommands>::SharedPtr command_publisher_;
    rclcpp::Publisher<qcar2_interfaces::msg::BooleanLeds>::SharedPtr led_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Joystick variables
    t_game_controller gamepad;
    t_error result;
    t_game_controller_states data;
    t_boolean is_new;

    t_uint8 controller_number = 0;
    t_uint16 buffer_size = 12;
    t_double deadzone[6] = {0.0};
    t_double saturation[6] = {0.0};
    t_boolean auto_center = false;
    t_uint16 max_force_feedback_effects = 0;
    t_double force_feedback_gain = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandPublisher>());
    rclcpp::shutdown();
    return 0;
}
