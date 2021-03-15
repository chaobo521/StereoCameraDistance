#include "lidar_perception.h"

using namespace std;
using namespace cv;

namespace lidar_perception{
lidar_perception::lidar_perception(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    pcl_subscriber_ = nh.subscribe("/points_raw", 1, &lidar_perception::PclCallback, this);
    marker_pub_box_ = nh.advertise<visualization_msgs::MarkerArray>("/lidar_perception/marker_box",1);
    marker_pub_tracinfo_ = nh.advertise<visualization_msgs::MarkerArray>("/lidar_perception/marker_info",1);
    marker_pub_trackinfo_vis_ = nh.advertise<visualization_msgs::MarkerArray>("/lidar_perception/marker_info_vis",1);
    marker_pub_track_ = nh.advertise<visualization_msgs::MarkerArray>("/lidar_perception/marker_track",1);
    pub_pcl_dist_ = nh.advertise<visualization_msgs::MarkerArray>("/lidar_perception/nearest_dist", 1);
    pub_pcl_cluster_ = nh.advertise<VPointCloud>("/lidar_perception/cluster_points",10);
    pub_pcl_cliped_ = nh.advertise<VPointCloud>("/lidar_perception/points",10);
    grid_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/lidar_perception/grid_map_topic", 1, this);

    nh.getParam("/lidar_perception/roi_x_min", x_min_);
	nh.getParam("/lidar_perception/roi_y_min", y_min_);
	nh.getParam("/lidar_perception/roi_z_min", z_min_);
    nh.getParam("/lidar_perception/roi_x_max", x_max_);
    nh.getParam("/lidar_perception/roi_y_max", y_max_);
    nh.getParam("/lidar_perception/roi_z_max", z_max_);
    nh.getParam("/lidar_perception/map_resolution", map_resolution_);
    nh.getParam("/lidar_perception/car_info/car_width", car_width_);
    nh.getParam("/lidar_perception/car_info/car_length", car_length_);

    map_x_min_ = 0;
	map_x_max_ = (int)((x_max_ - x_min_)/map_resolution_);
	map_y_min_ = 0;
	map_y_max_ = (int)((y_max_ - y_min_)/map_resolution_);

    max_marker_size_ = 0;
    picture_num_ = 0;
}

void lidar_perception::PclCallback(const VPointCloud::ConstPtr &rec)
{
    if(rec->points.size() > 0) {
        clock_t time_begin = clock();
        // tf coordinate transformation, from lidr to car

        // clip teh pcl

        //remove ground
        
        // cluster
        std::cout << "cluster done !" << std::endl;

        // display the cluster pointcloud

        // min_box
        std::cout << "minbox done !" << std::endl;

        // tracker
        std::cout << "tracker done !" << std::endl;

        // display the markers
        if(out_sensor_objects->objects.size() > 0) {
            ClearAllMarker();
            visualization_msgs::MarkerArray marker_array_box, marker_array_info, marker_array_info_vis, marker_array_track;

            visualization_msgs::Marker line_strip;
            line_strip.header.frame_id = "vehicle";
            line_strip.header.stamp = ros::Time::now();
            line_strip.ns = "sany::perception";
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;
            line_strip.scale.x = 0.05;
            line_strip.color.r = 0.0;
            line_strip.color.g = 1.0;
            line_strip.color.b = 0.0;
            line_strip.color.a = 1.0;
            line_strip.points.resize(5);
            int marker_id = 0;
            int trackerd_num = out_sensor_objects->objects.size();

            for(int i = 0; i < trackerd_num; i++) {
                geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
                p1.x = p5.x = out_sensor_objects->objects[i]->vertex1[0];
                p1.y = p5.y = out_sensor_objects->objects[i]->vertex1[1];
                p1.z = out_sensor_objects->objects[i]->min_height;
                p5.z = out_sensor_objects->objects[i]->max_height;
                p2.x = p6.x = out_sensor_objects->objects[i]->vertex2[0];
                p2.y = p6.y = out_sensor_objects->objects[i]->vertex2[1];
                p2.z = out_sensor_objects->objects[i]->min_height;
                p6.z = out_sensor_objects->objects[i]->max_height;
                p3.x = p7.x = out_sensor_objects->objects[i]->vertex3[0];
                p3.y = p7.y = out_sensor_objects->objects[i]->vertex3[1];
                p3.z = out_sensor_objects->objects[i]->min_height;
                p7.z = out_sensor_objects->objects[i]->max_height;
                p4.x = p8.x = out_sensor_objects->objects[i]->vertex4[0];
                p4.y = p8.y = out_sensor_objects->objects[i]->vertex4[1];
                p4.z = out_sensor_objects->objects[i]->min_height;
                p8.z = out_sensor_objects->objects[i]->max_height;
                line_strip.id = marker_id;//垂直z方向的方框，车底
                line_strip.points[0] = p1;
                line_strip.points[1] = p2;
                line_strip.points[2] = p4;
                line_strip.points[3] = p3;
                line_strip.points[4] = p1;
                marker_array_box.markers.push_back(line_strip);
                marker_id++;
                line_strip.id = marker_id;//垂直z方向的方框，车顶
                line_strip.points[0] = p5;
                line_strip.points[1] = p6;
                line_strip.points[2] = p8;
                line_strip.points[3] = p7;
                line_strip.points[4] = p5;
                marker_array_box.markers.push_back(line_strip);
                marker_id++;
                line_strip.id = marker_id;//垂直y方向的方框，车尾
                line_strip.points[0] = p1;
                line_strip.points[1] = p5;
                line_strip.points[2] = p7;
                line_strip.points[3] = p3;
                line_strip.points[4] = p1;
                marker_array_box.markers.push_back(line_strip);
                marker_id++;
                line_strip.id = marker_id;//垂直y方向的方框，车头
                line_strip.points[0] = p2;
                line_strip.points[1] = p6;
                line_strip.points[2] = p8;
                line_strip.points[3] = p4;
                line_strip.points[4] = p2;
                marker_array_box.markers.push_back(line_strip);
                marker_id++;
            }

            //for visualization
            visualization_msgs::Marker marker_txt_vis;
            marker_txt_vis.header.frame_id = "vehicle";
            marker_txt_vis.header.stamp = ros::Time::now();
            marker_txt_vis.ns = "sany::perception";
            marker_txt_vis.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker_txt_vis.scale.z = 0.3;
            marker_txt_vis.color.r = 1.0;
            marker_txt_vis.color.g = 1.0;
            marker_txt_vis.color.b = 1.0;
            marker_txt_vis.color.a = 1.0;
            for(int i = 0; i < trackerd_num; i++) {
                //由于雷达坐标系与车体坐标系定义不同，需要绕z轴旋转
                marker_txt_vis.pose.position.x = out_sensor_objects->objects[i]->anchor_point[0];
                marker_txt_vis.pose.position.y = out_sensor_objects->objects[i]->anchor_point[1];
                marker_txt_vis.pose.position.z = out_sensor_objects->objects[i]->max_height + 1.3;
                
                marker_txt_vis.id = i;
                marker_txt_vis.text = std::string("id: ") + std::to_string(out_sensor_objects->objects[i]->track_id) + "\n";
                double speed = sqrt(X2(out_sensor_objects->objects[i]->velocity[0]) + X2(out_sensor_objects->objects[i]->velocity[1]));
                marker_txt_vis.text += std::string("speed: ") + std::to_string(speed) + " m/s\n";
                marker_txt_vis.text += std::string("age: ") + std::to_string(static_cast<int>(out_sensor_objects->objects[i]->tracking_time)) + "s\n";
                marker_array_info_vis.markers.push_back(marker_txt_vis);
            }

            //for application
            visualization_msgs::Marker marker_txt;
            marker_txt.header.frame_id = "vehicle";
            marker_txt.header.stamp = ros::Time::now();
            marker_txt.ns = "sany::perception";
            marker_txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker_txt.scale.z = 0.3;
            marker_txt.color.r = 1.0;
            marker_txt.color.g = 1.0;
            marker_txt.color.b = 1.0;
            marker_txt.color.a = 1.0;
            for(int i = 0; i < trackerd_num; i++) {
                //由于雷达坐标系与车体坐标系定义不同，需要绕z轴旋转
                marker_txt.pose.position.x = out_sensor_objects->objects[i]->anchor_point[1];//存入障碍物的质心位置
                marker_txt.pose.position.y = -out_sensor_objects->objects[i]->anchor_point[0];
                marker_txt.pose.position.z = out_sensor_objects->objects[i]->max_height + 1.3;

                geometry_msgs::Point tmp;
                tmp.x = out_sensor_objects->objects[i]->length;
                tmp.y = out_sensor_objects->objects[i]->width;
                tmp.z = out_sensor_objects->objects[i]->height;
                marker_txt.points.push_back(tmp);//存入障碍物的长宽高
                
                marker_txt.id = i;
                marker_txt.text = std::string("id: ") + std::to_string(out_sensor_objects->objects[i]->track_id) + "\n";
                double speed = sqrt(X2(out_sensor_objects->objects[i]->velocity[0]) + X2(out_sensor_objects->objects[i]->velocity[1]));
                marker_txt.text += std::string("speed: ") + std::to_string(speed) + " m/s\n";
                marker_txt.text += std::string("age: ") + std::to_string(static_cast<int>(out_sensor_objects->objects[i]->tracking_time)) + "s\n";
                marker_txt.text += std::string("pos: (") + std::to_string(out_sensor_objects->objects[i]->anchor_point[0]) + ", " +
                                                           std::to_string(out_sensor_objects->objects[i]->anchor_point[1]) + ")\n";
                marker_txt.text += std::string("volume: (") + std::to_string(out_sensor_objects->objects[i]->length) + ", " +
                                                             std::to_string(out_sensor_objects->objects[i]->width) + ")\n";
                marker_array_info.markers.push_back(marker_txt);
                marker_txt.points.pop_back();
            }
            marker_pub_box_.publish(marker_array_box);
            marker_pub_tracinfo_.publish(marker_array_info);
            marker_pub_trackinfo_vis_.publish(marker_array_info_vis);
        }
        clock_t time_end = clock();
        std::cout << "all time = " << (double)(time_end-time_begin)/CLOCKS_PER_SEC << std::endl;
    }
}
}// namespace people_tracker