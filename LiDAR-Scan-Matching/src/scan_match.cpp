#include <sstream>
#include <string>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "yuwei_scan_matching/correspond.h"
#include "yuwei_scan_matching/transform.h"
#include "yuwei_scan_matching/visualization.h"

using namespace std;

const string TOPIC_SCAN = "/scan";
const string TOPIC_POS = "/scan_match_location";
const string TOPIC_RVIZ = "/scan_match_debug";
const string FRAME_POINTS = "laser";

const float RANGE_LIMIT = 10.0;
const float MAX_ITER = 2.0;
const float MIN_INFO = 0.1;
const float A = (1-MIN_INFO)/MAX_ITER/MAX_ITER;

class ScanProcessor : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    vector<Point> points_;
    vector<Point> transformed_points_;
    vector<Point> prev_points_;
    vector<Correspondence> corresponds_;
    vector< vector<int> > jump_table_;
    Transform prev_trans_, curr_trans_;

    std::unique_ptr<PointVisualizer> points_viz_;
    std::unique_ptr<CorrespondenceVisualizer> corr_viz_;

    geometry_msgs::msg::PoseStamped msg_;
    Eigen::Matrix3f global_tf_;
    std_msgs::msg::ColorRGBA col_;

    bool first_scan_;

public:
    ScanProcessor() : Node("scan_matcher"), curr_trans_(Transform()), first_scan_(true) {
        // Publishers
        pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(TOPIC_POS, 1);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(TOPIC_RVIZ, 1);
        
        // TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Visualizers
        points_viz_ = std::make_unique<PointVisualizer>(marker_pub_, "scan_match", FRAME_POINTS);
        corr_viz_ = std::make_unique<CorrespondenceVisualizer>(marker_pub_, "scan_match", FRAME_POINTS);

        // Initialize global transform to identity
        global_tf_ << 1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0;

        // Subscription
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            TOPIC_SCAN, 1, std::bind(&ScanProcessor::handleLaserScan, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Scan matcher initialized");
    }

    void handleLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        readScan(msg);

        // First scan - nothing to compare to
        if(first_scan_) {
            RCLCPP_INFO(this->get_logger(), "First Scan");
            prev_points_ = points_;
            first_scan_ = false;
            return;
        }

        // Visualize previous points
        col_.r = 1.0; col_.b = 0.0; col_.g = 0.0; col_.a = 1.0;
        points_viz_->addPoints(prev_points_, col_);

        int count = 0;
        computeJump(jump_table_, prev_points_);
        RCLCPP_INFO(this->get_logger(), "Starting Optimization");

        curr_trans_ = Transform(0.1, 0.1, 0.1);

        while (count < MAX_ITER && (curr_trans_ != prev_trans_ || count == 0)) {
            transformPoints(points_, curr_trans_, transformed_points_);

            // Find correspondence between points of current and previous frames
            getCorrespondence(prev_points_, transformed_points_, points_, jump_table_, corresponds_, A*count*count+MIN_INFO);

            prev_trans_ = curr_trans_;
            ++count;

            updateTransform(corresponds_, curr_trans_);
        }

        // Visualize transformed points
        col_.r = 0.0; col_.b = 0.0; col_.g = 1.0; col_.a = 1.0;
        points_viz_->addPoints(transformed_points_, col_);
        points_viz_->publishPoints();

        RCLCPP_INFO(this->get_logger(), "Optimization completed in %d iterations", count);

        // Update global transform
        global_tf_ = global_tf_ * curr_trans_.getMatrix();

        publishPos();
        prev_points_ = points_;
    }

    void readScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float range_min = msg->range_min;
        float range_max = msg->range_max;
        float angle_min = msg->angle_min;
        float angle_increment = msg->angle_increment;

        const vector<float>& ranges = msg->ranges;
        points_.clear();

        for (size_t i = 0; i < ranges.size(); ++i) {
            float range = ranges.at(i);
            if (range > RANGE_LIMIT) {
                continue;
            }
            if (!isnan(range) && range >= range_min && range <= range_max) {
                points_.push_back(Point(range, angle_min + angle_increment * i));
            }
        }
    }

    void publishPos() {
        msg_.pose.position.x = global_tf_(0,2);
        msg_.pose.position.y = global_tf_(1,2);
        msg_.pose.position.z = 0;
        
        // Convert rotation matrix to quaternion
        tf2::Matrix3x3 tf3d(
            static_cast<double>(global_tf_(0,0)), static_cast<double>(global_tf_(0,1)), 0,
            static_cast<double>(global_tf_(1,0)), static_cast<double>(global_tf_(1,1)), 0, 
            0, 0, 1
        );

        tf2::Quaternion q;
        tf3d.getRotation(q);
        
        msg_.pose.orientation = tf2::toMsg(q);
        msg_.header.frame_id = "map";
        msg_.header.stamp = this->get_clock()->now();
        pos_pub_->publish(msg_);

        // Broadcast transform
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "laser";
        transformStamped.transform.translation.x = global_tf_(0,2);
        transformStamped.transform.translation.y = global_tf_(1,2);
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(transformStamped);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanProcessor>());
    rclcpp::shutdown();
    return 0;
}
