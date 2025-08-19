#pragma once

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "yuwei_scan_matching/correspond.h"

using namespace std;

static int num_visuals = 0;

class PointVisualizer {
protected:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
    visualization_msgs::msg::Marker dots_;
    string ns_;
    string frame_id_;

public:
    PointVisualizer(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, string ns, string frame_id);
    void addPoints(vector<Point>& points, std_msgs::msg::ColorRGBA color);
    void publishPoints();
    ~PointVisualizer() {};
};

class CorrespondenceVisualizer {
protected:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
    visualization_msgs::msg::Marker line_list_;
    string ns_;
    string frame_id_;

public:
    CorrespondenceVisualizer(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, string ns, string frame_id);
    void addCorrespondences(vector<Correspondence> corresponds);
    void publishCorrespondences();
    ~CorrespondenceVisualizer() {};
};
