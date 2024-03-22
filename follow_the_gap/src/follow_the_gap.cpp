/* 

Authors: Georgios Tsakoumakis, Mayuresh Inamdar

------------------------------------------------------------ 
           COPYRIGHT AND LICENSING INFORMATION:
           NEWCASTLE UNIVERSITY - GPL 3.0
------------------------------------------------------------

This file is part of the F1TENTH internship conducted at Newcastle University.

This code is licensed under the GPL 3.0 license.

------------------------------------------------------------
              Follow The Gap Node

*/

#include "f110_reactive_methods/utility.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <map>
#include <string>

class FollowTheGap : public rclcpp::Node {
public:
    FollowTheGap() : Node("follow_the_gap"),
                     truncated_(false),
                     bubble_radius_(0.1), // Default value
                     max_accepted_distance_(3.0), // Default value
                     smoothing_filter_size_(5.0), // Default value
                     steering_angle_reactivity_(0.8), // Default value
                     truncated_coverage_angle_(3.14), // Default value
                     velocity_(0.0), // Default value
                     truncated_start_index_(0), // Default value
                     truncated_end_index_(0), // Default value
                     error_based_velocities_({{"high", 1.0}, {"medium", 3.0}, {"low", 4.0}}) // Default values
    {
        // SUBSCRIBER
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 100, std::bind(&FollowTheGap::scan_callback, this, std::placeholders::_1));

        // PUBLISHER
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "drive", 100);

        this->declare_parameter("bubble_radius", bubble_radius_);
        this->declare_parameter("smoothing_filter_size", smoothing_filter_size_);
        this->declare_parameter("truncated_coverage_angle", truncated_coverage_angle_);
        this->declare_parameter("velocity", velocity_);
        this->declare_parameter("max_accepted_distance", max_accepted_distance_);
        this->declare_parameter("error_based_velocities.high", error_based_velocities_["high"]);
        this->declare_parameter("error_based_velocities.medium", error_based_velocities_["medium"]);
        this->declare_parameter("error_based_velocities.low", error_based_velocities_["low"]);
        this->declare_parameter("steering_angle_reactivity", steering_angle_reactivity_);

        this->get_parameter("bubble_radius", bubble_radius_);
        this->get_parameter("smoothing_filter_size", smoothing_filter_size_);
        this->get_parameter("truncated_coverage_angle", truncated_coverage_angle_);
        this->get_parameter("velocity", velocity_);
        this->get_parameter("max_accepted_distance", max_accepted_distance_);
        this->get_parameter("error_based_velocities.high", error_based_velocities_["high"]);
        this->get_parameter("error_based_velocities.medium", error_based_velocities_["medium"]);
        this->get_parameter("error_based_velocities.low", error_based_velocities_["low"]);
        this->get_parameter("steering_angle_reactivity", steering_angle_reactivity_);
    }

    // Preprocess Lidar Scan to replace NaNs by zeros and find the closest point in the lidar scan
    std::vector<double> preprocess_lidar_scan(const sensor_msgs::msg::LaserScan &scan_msg) const
    {
        std::vector<double> filtered_ranges;
        for (size_t i = truncated_start_index_; i < truncated_end_index_; ++i)
        {
            if (std::isnan(scan_msg.ranges[i]))
            {
                filtered_ranges.push_back(0.0);
            }
            else if (scan_msg.ranges[i] > max_accepted_distance_ || std::isinf(scan_msg.ranges[i]))
            {
                filtered_ranges.push_back(max_accepted_distance_);
            }
            else
            {
                filtered_ranges.push_back(scan_msg.ranges[i]);
            }
        }
        return fgm::apply_smoothing_filter(filtered_ranges, smoothing_filter_size_);
    }

    // Get the Best Point in a Range of Max-Gap (according to some metric (In this case farthest point))
    size_t get_best_point(const std::vector<double> &filtered_ranges, int start_index, int end_index) const
    {
        return (start_index + end_index) / 2;
    }

    // Calculates the required steering angle based on the angles of the best point and the current heading of the vehicle
    double get_steering_angle_from_range_index(const sensor_msgs::msg::LaserScan &scan_msg,
                                               const size_t best_point_index,
                                               const double closest_value)
    {
        const size_t best_point_index_input_scan_frame = truncated_start_index_ + best_point_index;
        double best_point_steering_angle;
        // Case When Steering Angle is to be negative
        if (best_point_index_input_scan_frame < scan_msg.ranges.size() / 2)
        {
            best_point_steering_angle = -scan_msg.angle_increment *
                                        static_cast<double>(scan_msg.ranges.size() / 2.0 - best_point_index_input_scan_frame);
        }
        // Case When Steering Angle is to be positive
        else
        {
            best_point_steering_angle = scan_msg.angle_increment *
                                        static_cast<double>(best_point_index_input_scan_frame - scan_msg.ranges.size() / 2.0);
        }
        RCLCPP_DEBUG(get_logger(), "closest_value %f", closest_value);
        const auto distance_compensated_steering_angle =
            std::clamp((best_point_steering_angle * steering_angle_reactivity_) /
                           static_cast<double>(closest_value),
                       -1.57, 1.57);
        return distance_compensated_steering_angle;
    }

    // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        scan = *scan_msg;
        // Print the scan message
        RCLCPP_INFO(get_logger(), "Received scan message:");
        RCLCPP_INFO(get_logger(), "Angle Min: %f", scan_msg->angle_min);
        RCLCPP_INFO(get_logger(), "Angle Max: %f", scan_msg->angle_max);
        RCLCPP_INFO(get_logger(), "Angle Increment: %f", scan_msg->angle_increment);
        RCLCPP_INFO(get_logger(), "Time Increment: %f", scan_msg->time_increment);
        RCLCPP_INFO(get_logger(), "Scan Time: %f", scan_msg->scan_time);
        RCLCPP_INFO(get_logger(), "Range Min: %f", scan_msg->range_min);
        RCLCPP_INFO(get_logger(), "Range Max: %f", scan_msg->range_max);

        if (!truncated_)
        {
            RCLCPP_INFO(get_logger(), "input scan start angle = %f", scan.angle_min);
            RCLCPP_INFO(get_logger(), "input scan end angle = %f", scan.angle_max);
            RCLCPP_INFO(get_logger(), "input scan start index = %u", 0);
            RCLCPP_INFO(get_logger(), "input scan end index = %u", static_cast<unsigned int>(scan.ranges.size()));

            const auto truncated_indices =
                fgm::truncated_start_and_end_indices(scan, truncated_coverage_angle_);
            truncated_start_index_ = truncated_indices.first;
            truncated_end_index_ = truncated_indices.second;
            truncated_ = true;

            RCLCPP_INFO(get_logger(), "truncated scan start angle = %f", -truncated_coverage_angle_ / 2);
            RCLCPP_INFO(get_logger(), "truncated scan end angle = %f", truncated_coverage_angle_ / 2);
            RCLCPP_INFO(get_logger(), "truncated start index = %u", static_cast<unsigned int>(truncated_start_index_));
            RCLCPP_INFO(get_logger(), "truncated end index = %u", static_cast<unsigned int>(truncated_end_index_));
        }

        // Pre-Process (zero out NaNs and filter)
        auto filtered_ranges = preprocess_lidar_scan(scan);
        RCLCPP_INFO(get_logger(), "Created Filtered Ranges");

        // find the closest point to LiDAR
        const size_t closest_index = fgm::minimum_element_index(filtered_ranges);
        RCLCPP_INFO(get_logger(), "Closest Point Index = %u", static_cast<unsigned int>(closest_index + truncated_start_index_));

        const auto closest_range = filtered_ranges[closest_index];
        RCLCPP_INFO(get_logger(), "Closest Point Value = %f", filtered_ranges[closest_index]);

        // Eliminate all points inside 'bubble' (set them to zero)
        fgm::zero_out_safety_bubble(&filtered_ranges, closest_index, bubble_radius_);
        RCLCPP_INFO(get_logger(), "Zeroed out the Bubble Region");

        // Find max length gap
        const auto [start_index, end_index] = fgm::find_largest_nonzero_sequence(filtered_ranges);
        RCLCPP_INFO(get_logger(), "Max Gap Start Index = %u", static_cast<unsigned int>(start_index + truncated_start_index_));
        RCLCPP_INFO(get_logger(), "Max Gap End Index = %u", static_cast<unsigned int>(end_index + truncated_start_index_));

        // Find the best point in the gap
        const size_t best_point_index = get_best_point(filtered_ranges, start_index, end_index);
        RCLCPP_INFO(get_logger(), "Best Point = %u", static_cast<unsigned int>(best_point_index + truncated_start_index_));

        const double steering_angle = get_steering_angle_from_range_index(scan, best_point_index, closest_range);
        RCLCPP_INFO(get_logger(), "Publish Steering Angle = %f", steering_angle);

        // Publish Drive message
        auto drive_msg = std::make_shared<ackermann_msgs::msg::AckermannDriveStamped>();
        drive_msg->header.stamp = now();
        drive_msg->header.frame_id = "laser";
        drive_msg->drive.steering_angle = steering_angle;
        if (std::abs(steering_angle) > 0.349)
        {
            drive_msg->drive.speed = error_based_velocities_["high"];
        }
        else if (std::abs(steering_angle) > 0.174)
        {
            drive_msg->drive.speed = error_based_velocities_["medium"];
        }
        else
        {
            drive_msg->drive.speed = error_based_velocities_["low"];
        }
        drive_pub_->publish(*drive_msg);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    sensor_msgs::msg::LaserScan scan;

    double bubble_radius_;
    double max_accepted_distance_;
    double smoothing_filter_size_;
    double steering_angle_reactivity_;

    bool truncated_;
    double truncated_coverage_angle_;
    size_t truncated_start_index_;
    size_t truncated_end_index_;

    double velocity_;

    std::map<std::string, double> error_based_velocities_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowTheGap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

