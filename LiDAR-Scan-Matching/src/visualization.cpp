#include "yuwei_scan_matching/visualization.h"

PointVisualizer::PointVisualizer(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, string ns, string frame_id) 
    : pub_(pub), ns_(ns), frame_id_(frame_id) {
    dots_.header.frame_id = frame_id;
    dots_.ns = ns;
    dots_.action = visualization_msgs::msg::Marker::ADD;
    dots_.pose.orientation.w = 1.0;
    dots_.id = num_visuals;
    dots_.type = visualization_msgs::msg::Marker::POINTS;
    dots_.scale.x = dots_.scale.y = 0.08;
    ++num_visuals;
}

void PointVisualizer::addPoints(vector<Point>& points, std_msgs::msg::ColorRGBA color) {
    for (Point p : points) {
        dots_.points.push_back(p.getPoint());
        dots_.colors.push_back(color);
    }
}

void PointVisualizer::publishPoints() {
    dots_.header.stamp = rclcpp::Clock().now();
    pub_->publish(dots_);
    dots_.points.clear();
    dots_.colors.clear();
}

CorrespondenceVisualizer::CorrespondenceVisualizer(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, string ns, string frame_id) 
    : pub_(pub), ns_(ns), frame_id_(frame_id) {
    line_list_.header.frame_id = frame_id;
    line_list_.ns = ns;
    line_list_.action = visualization_msgs::msg::Marker::ADD;
    line_list_.pose.orientation.w = 1.0;
    line_list_.id = num_visuals;
    line_list_.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_list_.scale.x = 0.001;
    num_visuals++;
}

void CorrespondenceVisualizer::addCorrespondences(vector<Correspondence> correspondences) {
    std_msgs::msg::ColorRGBA col;
    col.r = 1.0; col.b = 0.0; col.g = 0.0; col.a = 1.0;
    for (Correspondence c : correspondences) {
        line_list_.points.push_back(c.p->getPoint());
        line_list_.colors.push_back(col);
        line_list_.points.push_back(c.getPiGeo());
        line_list_.colors.push_back(col);
    }
}

void CorrespondenceVisualizer::publishCorrespondences() {
    line_list_.header.stamp = rclcpp::Clock().now();
    pub_->publish(line_list_);
    line_list_.points.clear();
    line_list_.colors.clear();
}
