#ifndef LIDAR_PERSEPTION_H_
#define LIDAR_PERSEPTION_H_

#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose2D.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <sstream>
#include <memory>
#include <time.h>
#include <nav_msgs/OccupancyGrid.h>

namespace lidar_perception {
class lidar_perception{
public:
    lidar_perception(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~lidar_perception() {};
private:
    void PclCallback(const VPointCloud::ConstPtr &rec);
    void ClearAllMarker();
    void makeGridMap(const PointCloudPtr &cloudtogrid);

    ros::Subscriber pcl_subscriber_;
    ros::Publisher marker_pub_box_, marker_pub_trackinfo_vis_, marker_pub_tracinfo_, marker_pub_track_;
    ros::Publisher pub_pcl_cluster_, pub_pcl_cliped_;
    ros::Publisher grid_map_pub_;
    ros::Publisher pub_pcl_dist_;

    tf::TransformListener lidar2base_;

    int max_marker_size_;
    int picture_num_;

    bool is_using_path_roi_ = true;
    double speed_;
    double steer_;
    double wheelbase_ = 2.84;
    double front_overhang_ = 1.1;
    double vehicle_x_ = 0.0;
    double vehicle_y_ = 0.0;
    double vehicle_theta_ = 0.0;

    int map_x_min_;
    int map_x_max_;
    int map_y_min_;
    int map_y_max_;
    double map_resolution_;
    double car_width_;
    double car_length_;
    double x_min_;
    double x_max_;
    double y_min_;
    double y_max_;
    double z_min_; 
    double z_max_;
};
}
#endif