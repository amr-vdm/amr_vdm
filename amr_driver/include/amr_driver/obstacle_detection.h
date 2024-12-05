#ifndef _AMR_V3_DRIVER__OBSTACLE_DETECTION_H_
#define _AMR_V3_DRIVER__OBSTACLE_DETECTION_H_

#include <cmath>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>

namespace obstacle_detection
{

enum ObstacleFiled
{
    NO_OBSTACLE,
    OBSTACLE
};

class ObstacleDetection
{

public:
    ObstacleDetection(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~ObstacleDetection(){}

    void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void pubObstacleField(const int msg);

private:
    ros::Publisher pub_obstacle_field_;
    ros::Subscriber sub_laser_scan_;

    double min_range_;
    double max_range_;
    int threshold_;
    std::string laser_scan_topic_;

};    
} // namespace obstacle_detection


#endif // _AMR_V3_DRIVER__OBSTACLE_DETECTION_H_