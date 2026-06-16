#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

using namespace std::chrono_literals;

class InitialPosePublisher : public rclcpp::Node {
public:
    InitialPosePublisher() : Node("initial_pose_publisher") {
        this->declare_parameter<std::string>("map_frame", "map");
        
        // Use dynamically typed parameter descriptors to accept both double and string types
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.dynamic_typing = true;
        this->declare_parameter("x", rclcpp::ParameterValue(0.0), desc);
        this->declare_parameter("y", rclcpp::ParameterValue(0.0), desc);
        this->declare_parameter("z", rclcpp::ParameterValue(0.0), desc);
        this->declare_parameter("roll", rclcpp::ParameterValue(0.0), desc);
        this->declare_parameter("pitch", rclcpp::ParameterValue(0.0), desc);
        this->declare_parameter("yaw", rclcpp::ParameterValue(0.0), desc);

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

        // Periodically publish the initial seed coordinates.
        // Once the mapping node successfully relocalizes, it will set first_frame_relocalized = true
        // and ignore all subsequent publications.
        timer_ = this->create_wall_timer(500ms, std::bind(&InitialPosePublisher::publish_pose, this));
        
        RCLCPP_INFO(this->get_logger(), "Initial Pose Node Started. Seeding coordinate state periodically...");
    }

private:
    double get_double_param(const std::string& name) {
        if (!this->has_parameter(name)) return 0.0;
        auto param = this->get_parameter(name);
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            return param.as_double();
        } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            try {
                return std::stod(param.as_string());
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse parameter %s as double: %s", name.c_str(), param.as_string().c_str());
            }
        } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            return static_cast<double>(param.as_int());
        }
        return 0.0;
    }

    void publish_pose() {
        auto message = geometry_msgs::msg::PoseWithCovarianceStamped();

        std::string map_frame = this->get_parameter("map_frame").as_string();
        double x = get_double_param("x");
        double y = get_double_param("y");
        double z = get_double_param("z");
        double roll = get_double_param("roll");
        double pitch = get_double_param("pitch");
        double yaw = get_double_param("yaw");

        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = map_frame;

        message.pose.pose.position.x = x;
        message.pose.pose.position.y = y;
        message.pose.pose.position.z = z;

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        message.pose.pose.orientation.x = q.x();
        message.pose.pose.orientation.y = q.y();
        message.pose.pose.orientation.z = q.z();
        message.pose.pose.orientation.w = q.w();

        message.pose.covariance[0]  = 0.05;
        message.pose.covariance[7]  = 0.05;
        message.pose.covariance[14] = 0.05;
        message.pose.covariance[21] = 0.02;
        message.pose.covariance[28] = 0.02;
        message.pose.covariance[35] = 0.02;

        RCLCPP_INFO(this->get_logger(), "Publishing Initial Pose Seed: [x: %.2f, y: %.2f, z: %.2f]", x, y, z);
        pose_pub_->publish(message);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialPosePublisher>());
    rclcpp::shutdown();
    return 0;
}