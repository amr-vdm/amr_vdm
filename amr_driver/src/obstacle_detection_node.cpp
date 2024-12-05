#include <amr_driver/obstacle_detection.h>

namespace obstacle_detection
{

ObstacleDetection::ObstacleDetection(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    initialize(nh, pnh);
    ROS_INFO("Intialized Obstacle detection.");
}

void ObstacleDetection::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    pnh.param("min_range", min_range_, 0.2);
    pnh.param("max_range", max_range_, 0.35);
    pnh.param("noise_threshold", threshold_, 5);
    pnh.param("laser_scan_topic", laser_scan_topic_, std::string("scan"));

    pub_obstacle_field_ = nh.advertise<std_msgs::Int16>("obstacle_field", 1);

    sub_laser_scan_ = nh.subscribe(laser_scan_topic_, 10, &ObstacleDetection::laserScanCallback, this);
}

void ObstacleDetection::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::vector<float> laser_data = msg->ranges;
    int count_obstacles = 0;

    for (const auto& range_value : laser_data) {
        if (!std::isnan(range_value) && range_value >= static_cast<float>(min_range_)
            && range_value <= static_cast<float>(max_range_)) {
            count_obstacles++;
        }

        if (count_obstacles > threshold_) break;
    }
    if (count_obstacles > threshold_) {
        pubObstacleField(ObstacleFiled::OBSTACLE);
        // ROS_INFO("Detect obstacle in field!");
    }
    else {
        pubObstacleField(ObstacleFiled::NO_OBSTACLE);
        // ROS_INFO("No obstacle in field.");
    }
}

void ObstacleDetection::pubObstacleField(const int data)
{
    std_msgs::Int16 msg;
    msg.data = data;
    pub_obstacle_field_.publish(msg);
}
} // namespace obstacle_detection

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_detection_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    obstacle_detection::ObstacleDetection obstacle_detection(nh, pnh);
    ros::spin();

    return 0;
}
