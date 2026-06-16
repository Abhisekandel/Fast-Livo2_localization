#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class InitialPosePublisher : public rclcpp::Node {
public:
    InitialPosePublisher() : Node("initial_pose_publisher") {
        this->declare_parameter<std::string>("map_frame", "camera_init");
        
        // Declare parameter to accept your 16-element flattened 4x4 matrix
        // Defaults to Identity Matrix (0,0,0 position, 0 rotation)
        this->declare_parameter<std::vector<double>>("localization_matrix", {
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        });

        this->declare_parameter<double>("x", 0.0);
        this->declare_parameter<double>("y", 0.0);
        this->declare_parameter<double>("z", 0.0);
        this->declare_parameter<double>("roll", 0.0);
        this->declare_parameter<double>("pitch", 0.0);
        this->declare_parameter<double>("yaw", 0.0);

        // Use transient_local so late-joining subscribers receive the last published message
        auto qos = rclcpp::QoS(10).transient_local();
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/initialpose", qos);

        // Check if a non-zero pose was configured
        bool has_pose = false;
        auto mat = this->get_parameter("localization_matrix").as_double_array();
        std::vector<double> identity = {
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        };
        if (mat != identity) {
            has_pose = true;
        } else {
            double x = this->get_parameter("x").as_double();
            double y = this->get_parameter("y").as_double();
            double z = this->get_parameter("z").as_double();
            double roll = this->get_parameter("roll").as_double();
            double pitch = this->get_parameter("pitch").as_double();
            double yaw = this->get_parameter("yaw").as_double();
            if (x != 0.0 || y != 0.0 || z != 0.0 || roll != 0.0 || pitch != 0.0 || yaw != 0.0) {
                has_pose = true;
            }
        }

        if (has_pose) {
            // Build the pose message once
            pose_msg_ = build_pose_message();
            RCLCPP_INFO(this->get_logger(), "Initial pose configured. Publishing every 1s until subscriber connects (max 60s).");
            // Publish repeatedly every 1 second to survive DDS discovery delays
            timer_ = this->create_wall_timer(1000ms, std::bind(&InitialPosePublisher::publish_tick, this));
        } else {
            RCLCPP_INFO(this->get_logger(), "No initial pose configured (all zeros). System will start from origin.");
        }
    }

private:
    geometry_msgs::msg::PoseStamped build_pose_message() {
        auto mat = this->get_parameter("localization_matrix").as_double_array();
        std::vector<double> identity = {
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        };

        auto message = geometry_msgs::msg::PoseStamped();
        std::string map_frame = this->get_parameter("map_frame").as_string();
        message.header.frame_id = map_frame;

        if (mat == identity) {
            double x = this->get_parameter("x").as_double();
            double y = this->get_parameter("y").as_double();
            double z = this->get_parameter("z").as_double();
            double roll = this->get_parameter("roll").as_double();
            double pitch = this->get_parameter("pitch").as_double();
            double yaw = this->get_parameter("yaw").as_double();

            message.pose.position.x = x;
            message.pose.position.y = y;
            message.pose.position.z = z;

            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            message.pose.orientation.x = q.x();
            message.pose.orientation.y = q.y();
            message.pose.orientation.z = q.z();
            message.pose.orientation.w = q.w();

            RCLCPP_INFO(this->get_logger(), "Pose from xyz/rpy: [x: %.3f, y: %.3f, z: %.3f, r: %.3f, p: %.3f, y: %.3f]", 
                        x, y, z, roll, pitch, yaw);
        } else {
            if (mat.size() != 16) {
                RCLCPP_ERROR(this->get_logger(), "Matrix error! 'localization_matrix' must contain exactly 16 elements.");
                return message;
            }

            message.pose.position.x = mat[3];
            message.pose.position.y = mat[7];
            message.pose.position.z = mat[11];

            tf2::Matrix3x3 rotation_matrix(
                mat[0], mat[1], mat[2],
                mat[4], mat[5], mat[6],
                mat[8], mat[9], mat[10]
            );

            tf2::Quaternion q;
            rotation_matrix.getRotation(q);

            message.pose.orientation.x = q.x();
            message.pose.orientation.y = q.y();
            message.pose.orientation.z = q.z();
            message.pose.orientation.w = q.w();

            RCLCPP_INFO(this->get_logger(), "Pose from matrix: [x: %.3f, y: %.3f, z: %.3f]", 
                        message.pose.position.x, message.pose.position.y, message.pose.position.z);
        }
        return message;
    }

    void publish_tick() {
        publish_count_++;
        
        // Stop after 60 publishes (60 seconds)
        if (publish_count_ > 60) {
            timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Stopped publishing initial pose after 60 attempts.");
            return;
        }

        // Check if subscriber is connected
        size_t sub_count = pose_pub_->get_subscription_count();
        
        // Update timestamp and publish
        pose_msg_.header.stamp = this->get_clock()->now();
        pose_pub_->publish(pose_msg_);
        
        if (sub_count > 0) {
            RCLCPP_INFO(this->get_logger(), "Published initial pose [%.3f, %.3f, %.3f] (%zu subscriber(s), attempt %d).",
                        pose_msg_.pose.position.x, pose_msg_.pose.position.y, pose_msg_.pose.position.z,
                        sub_count, publish_count_);
            // Keep publishing a few more times after subscriber connects to be safe, then stop
            if (publish_count_ >= 5) {
                timer_->cancel();
                RCLCPP_INFO(this->get_logger(), "Subscriber confirmed. Stopping publisher.");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Publishing initial pose (attempt %d, waiting for subscriber)...", publish_count_);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseStamped pose_msg_;
    int publish_count_ = 0;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialPosePublisher>());
    rclcpp::shutdown();
    return 0;
}