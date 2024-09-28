/*  Copyright (C) Amir Darwesh
 * 
 *  License: Modified BSD Software License 
 */


#include <string>
#include <iostream>
#include <algorithm>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common_headers.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Pose.h>

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class HeightmapToCostMap
{
public:
    HeightmapToCostMap();
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void generate_costmap();

    bool DO_INFLATION = true; // true
    float RESOLUTION_ = 0.1; // [m / cell]
    float MAP_MIN_X =  -5; // map min x position
    float MAP_MAX_X =  15; // map max x position
    float MAP_MIN_Y = -10; // map min y position
    float MAP_MAX_Y =  10; // map max y position

    float INFLATION_RADIUS = 0.2; // [m] size of inflation
    float INFLATION_RES    = RESOLUTION_; // [m] resolution of inflation
    int INFLATION_BINS     = INFLATION_RADIUS / INFLATION_RES;

private:
    ros::NodeHandle nh_;
    std::string cloud_topic_; //default input
    std::string map_topic_;
    ros::Subscriber sub_;
    ros::Publisher cost_map_pub_;

    // Varialbes
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyz;

    bool bGetPoint = false;
};

HeightmapToCostMap::HeightmapToCostMap() : cloud_topic_("/points/velodyne_obstacles"), map_topic_("/map/local_map/obstacle")
{
    sub_ = nh_.subscribe(cloud_topic_, 30, &HeightmapToCostMap::cloud_cb, this);

    cost_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_topic_, 10);

    //print some info about the node
    ROS_INFO("[HeightmapToCostMap] Loaded!");
}

void HeightmapToCostMap::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    // Update point cloud data
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyz_(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud_xyz_); // conver to pcl object
    cloud_xyz = cloud_xyz_;
    bGetPoint = true;
}

void HeightmapToCostMap::generate_costmap()
{
    // Generate costmap after getting point cloud data
    if (bGetPoint)
    {
        // get bounds
        Eigen::Vector4f min_pt;
        Eigen::Vector4f max_pt;

        int width_ = int(MAP_MAX_X - MAP_MIN_X + 0.5f);
        width_ = int(width_ / RESOLUTION_ + 0.5f);

        int height_ = int(MAP_MAX_Y - MAP_MIN_Y + 0.5f);
        height_ = int(height_ / RESOLUTION_ + 0.5f);

        // ROS_INFO_THROTTLE(0.5, "Image Dimensions w %d x h %d", width_, height_);
        nav_msgs::MapMetaData mapMeta;

        mapMeta.resolution = RESOLUTION_;
        mapMeta.width = width_;
        mapMeta.height = height_;

        geometry_msgs::Pose oPose;
        oPose.position.x = MAP_MIN_X - RESOLUTION_/2;
        oPose.position.y = MAP_MIN_Y - RESOLUTION_/2;
        mapMeta.origin = oPose;

        nav_msgs::OccupancyGrid oMap;
        oMap.info = mapMeta;
        oMap.data.resize(width_ * height_);
        oMap.header.frame_id = cloud_xyz->header.frame_id; // base_link cloud_xyz->header.frame_id
        oMap.header.stamp = ros::Time::now();

        // Generating cost map w.r.t. obstacles
        for (pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud_xyz->begin(); it != cloud_xyz->end(); it++)
        {
            // check for nan points && min/max points
            if ((!(isnan(it->x) | isnan(it->y))) && (it->x > MAP_MIN_X && it->x < MAP_MAX_X) && (it->y > MAP_MIN_Y && it->y < MAP_MAX_Y))
            {
                int x = int((it->x - MAP_MIN_X) / RESOLUTION_);
                int y = int((it->y - MAP_MIN_Y) / RESOLUTION_);
                // ROS_INFO("X %d, Y %d, x %f ,y %f", x, y, it->x, it->y);
                if (x < width_ && y < height_)
                {
                    if (DO_INFLATION)
                    {
                        double current_x = it->x;
                        double current_y = it->y;

                        for (int i=0; i < INFLATION_BINS; i++)
                        {
                            for (int j=0; j < INFLATION_BINS; j++)
                            {
                                double dx = i * INFLATION_RES - 0.5*INFLATION_RADIUS;
                                double dy = j * INFLATION_RES - 0.5*INFLATION_RADIUS;
                                double dist = sqrt(pow(dx, 2.0) + pow(dy, 2.0));
                                double map_value = 100 - std::min(50.0, 20 * dist);

                                double padded_x = current_x + dx;
                                double padded_y = current_y + dy;

                                double padded_map_x = int((padded_x - MAP_MIN_X) / RESOLUTION_);
                                double padded_map_y = int((padded_y - MAP_MIN_Y) / RESOLUTION_);
                                if (padded_map_x < width_ && padded_map_x >= 0 && padded_map_y < height_ && padded_map_y >= 0)
                                {
                                    // update to larger map value
                                    double current_map_value = oMap.data[MAP_IDX(width_, padded_map_x, padded_map_y)];
                                    oMap.data[MAP_IDX(width_, padded_map_x, padded_map_y)] = std::max(map_value, current_map_value);
                                }
                            }
                        }
                    }
                    // No inflation
                    else
                    {
                        oMap.data[MAP_IDX(width_, x, y)] = 100;
                    }
                    
                }
            }
        }

        // Publish cost map
        cost_map_pub_.publish(oMap);
    }

    else
    {
        ROS_INFO("No point cloud yet!!!");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heightmap_to_costmap");

    HeightmapToCostMap hcm; //this loads up the node
    ros::Rate rate(50);
    while (ros::ok())
    {
        ros::spinOnce(); //where she stops nobody knows
        hcm.generate_costmap();
        rate.sleep();
    }
}
