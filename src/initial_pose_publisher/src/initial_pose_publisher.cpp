#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" // Switched to flat PoseStamped for FAST-LIO matching
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class InitialPosePublisher : public rclcpp::Node {
public:
    InitialPosePublisher() : Node("initial_pose_publisher") {
        this->declare_parameter<std::string>("map_frame", "camera_init"); // Matches your FAST-LIO log frame
        
        // Declare parameter to accept your 16-element flattened 4x4 matrix
        // Defaults to Identity Matrix (0,0,0 position, 0 rotation)
        this->declare_parameter<std::vector<double>>("localization_matrix", {
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        });

        // FAST-LIO typically uses standard PoseStamped tracking over covariance templates
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/initialpose", 10);

        // STAGE 1: Fire the (0,0,0) Identity matrix IMMEDIATELY on startup
        RCLCPP_INFO(this->get_logger(), "STAGE 1: Resetting localization engine baseline to (0,0,0)...");
        publish_identity_pose();

        // STAGE 2: Set a one-shot wall timer to apply the true matrix calculation after 2 seconds
        timer_ = this->create_wall_timer(2000ms, std::bind(&InitialPosePublisher::publish_localization_pose, this));
    }

private:
    // Publishes a clean 0,0,0 baseline map anchor
    void publish_identity_pose() {
        auto message = geometry_msgs::msg::PoseStamped();
        std::string map_frame = this->get_parameter("map_frame").as_string();

        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = map_frame;

        message.pose.position.x = 0.0;
        message.pose.position.y = 0.0;
        message.pose.position.z = 0.0;
        message.pose.orientation.w = 1.0; // Identity Quaternion

        pose_pub_->publish(message);
    }

    // Extracts your calculated 3D transformation matrix and overrides the position
    void publish_localization_pose() {
        timer_->cancel(); // Cancel timer so it acts as a one-shot trigger
        
        auto mat = this->get_parameter("localization_matrix").as_double_array();
        if (mat.size() != 16) {
            RCLCPP_ERROR(this->get_logger(), "Matrix error! 'localization_matrix' array must contain exactly 16 elements.");
            return;
        }

        auto message = geometry_msgs::msg::PoseStamped();
        std::string map_frame = this->get_parameter("map_frame").as_string();

        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = map_frame;

        // 1. Extract translation column elements (Row-Major format indices: 3, 7, 11)
        message.pose.position.x = mat[3];
        message.pose.position.y = mat[7];
        message.pose.position.z = mat[11];

        // 2. Reconstruct the 3x3 rotation portion to transform into a solid Quaternion block
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

        RCLCPP_INFO(this->get_logger(), "STAGE 2: Applying calculated matrix shift -> [x: %.3f, y: %.3f, z: %.3f]", 
                    message.pose.position.x, message.pose.position.y, message.pose.position.z);
        
        pose_pub_->publish(message);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialPosePublisher>());
    rclcpp::shutdown();
    return 0;
}
"""

mat0 mat1 mat2 x(mat3)
mat4 mat5 mat6 y(mat7)
mat8 mat9 mat10 z(mat11)
0    0    0      1

"""