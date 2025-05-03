#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class QCar2OdomBroadcaster : public rclcpp::Node {
public:
  QCar2OdomBroadcaster()
  : Node("qcar2_odometry_broadcaster"), x_(0.0), y_(0.0), theta_(0.0) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    last_time_ = this->now();

    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/qcar2_joint", 10,
      std::bind(&QCar2OdomBroadcaster::joint_callback, this, std::placeholders::_1));
  }

private:
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->velocity.empty()) return;

    double v = msg->velocity[0];  // linear speed from motor
    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();

    x_ += v * std::cos(theta_) * dt;
    y_ += v * std::sin(theta_) * dt;

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = now;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = x_;
    odom_tf.transform.translation.y = y_;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(odom_tf);

    last_time_ = now;
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Time last_time_;
  double x_, y_, theta_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QCar2OdomBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
