/*  Copyright (C) 2010 UT-Austin &  Austin Robot Technology,
 *  David Claridge, Michael Quinlan
 *  Copyright (C) 2012 Jack O'Quin
 * 
 *  License: Modified BSD Software License 
 */

/** @file

    @brief ROS class for detecting obstacles in a point cloud.

   This class produces a point cloud containing all points that lie on
   an obstacle and are taller than the @c height_threshold parameter.

Subscribes:

- @b velodyne_points [sensor_msgs::PointCloud2] data from one
  revolution of the Velodyne LIDAR

Publishes:

- @b veloydne_obstacles [sensor_msgs::PointCloud2] grid cells that
  contain an obstacle

- @b veloydne_clear [sensor_msgs::PointCloud2] grid cells with no
  obstacles


@author David Claridge, Michael Quinlan 

*/

#include <local_costmap_generator/heightmap.h>

namespace local_costmap_generator {

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))

HeightMap::HeightMap(ros::NodeHandle node, ros::NodeHandle priv_nh)
{
  // get parameters using private node handle
  priv_nh.param("cell_size", m_per_cell_, 0.1); // [m / cell]
  priv_nh.param("full_clouds", full_clouds_, false);
  priv_nh.param("grid_dimensions", grid_dim_, 200); // [cell] size of map; 200 cell = 20 [m] / 0.1 [m/cell]; 20 is calculated from MAP_MAX_X - MAP_MIN_X at 'heightmap_to_costmap.cpp'
  priv_nh.param("height_threshold", height_diff_threshold_, 0.25); // [m]
  
  ROS_INFO_STREAM("height map parameters: "
                  << grid_dim_ << "x" << grid_dim_ << ", "
                  << m_per_cell_ << "m cells, "
                  << height_diff_threshold_ << "m threshold, "
                  << (full_clouds_? "": "not ") << "publishing full clouds");

  // Set up publishers  
  obstacle_publisher_ = node.advertise<VPointCloud>("/points/velodyne_obstacles",1);
  clear_publisher_ = node.advertise<VPointCloud>("/points/velodyne_clear",1);  

  // subscribe to Velodyne data points
  // TODO: find and change the topic name of the point cloud data
  velodyne_scan_ = node.subscribe("/camera/depth/points", 10,
                                  &HeightMap::processData, this,
                                  ros::TransportHints().tcpNoDelay(true));
}

HeightMap::~HeightMap() {}

void HeightMap::constructFullClouds(const VPointCloud::ConstPtr &scan,
                                    unsigned npoints, size_t &obs_count,
                                    size_t &empty_count)
{
  float min[grid_dim_][grid_dim_];
  float max[grid_dim_][grid_dim_];
  bool init[grid_dim_][grid_dim_];
  memset(&init, 0, grid_dim_*grid_dim_);
  
  // build height map
  // for realsense camera, x <- z; y <- -y; z <- -x
  for (unsigned i = 0; i < npoints; ++i) {
    // TODO: convert position of points from camera to local coordinate
    // - camera coordinate (x,y,z): scan->points[i].x, scan->points[i].y, scan->points[i].z
    // - local coordinate (x,y,z): local_x, local_y, local_z
    double local_x = scan->points[i].x; // currently, wrong transform. you need to fix this!
    double local_y = scan->points[i].y; // currently, wrong transform. you need to fix this!
    double local_z = scan->points[i].z; // currently, wrong transform. you need to fix this!

    int x = ((grid_dim_/2)+(local_x)/m_per_cell_); // ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
    int y = ((grid_dim_/2)+(local_y)/m_per_cell_); // ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_) {
      if (!init[x][y]) {
        min[x][y] = local_z; // scan->points[i].z
        max[x][y] = local_z; // scan->points[i].z
        init[x][y] = true;
      } else {
        min[x][y] = MIN(min[x][y], local_z); // scan->points[i].z
        max[x][y] = MAX(max[x][y], local_z); // scan->points[i].z
      }
    }
  }

  // display points where map has height-difference > threshold
  for (unsigned i = 0; i < npoints; ++i) {
    // TODO: convert position of points from camera to local coordinate
    // - camera coordinate (x,y,z): scan->points[i].x, scan->points[i].y, scan->points[i].z
    // - local coordinate (x,y,z): local_x, local_y, local_z
    double local_x = scan->points[i].x; // currently, wrong transform. you need to fix this!
    double local_y = scan->points[i].y; // currently, wrong transform. you need to fix this!
    double local_z = scan->points[i].z; // currently, wrong transform. you need to fix this!

    int x = ((grid_dim_/2)+(local_x)/m_per_cell_); // ((grid_dim_/2)+scan->points[i].x/m_per_cell_)
    int y = ((grid_dim_/2)+(local_y)/m_per_cell_); // ((grid_dim_/2)+scan->points[i].y/m_per_cell_)
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]) {
      if ((max[x][y] - min[x][y] > height_diff_threshold_) ) {   
        obstacle_cloud_.points[obs_count].x = local_x;
        obstacle_cloud_.points[obs_count].y = local_y;
        obstacle_cloud_.points[obs_count].z = local_z;
        //obstacle_cloud_.channels[0].values[obs_count] = (float) scan->points[i].intensity;
        obs_count++;
      } else {
        clear_cloud_.points[empty_count].x = local_x;
        clear_cloud_.points[empty_count].y = local_y;
        clear_cloud_.points[empty_count].z = local_z;
        //clear_cloud_.channels[0].values[empty_count] = (float) scan->points[i].intensity;
        empty_count++;
      }
    }
  }
}

void HeightMap::constructGridClouds(const VPointCloud::ConstPtr &scan,
                                    unsigned npoints, size_t &obs_count,
                                    size_t &empty_count)
{
  float min[grid_dim_][grid_dim_];
  float max[grid_dim_][grid_dim_];
  float num_obs[grid_dim_][grid_dim_];
  float num_clear[grid_dim_][grid_dim_];
  bool init[grid_dim_][grid_dim_];
  
  for (int x = 0; x < grid_dim_; x++) {
    for (int y = 0; y < grid_dim_; y++) {
      init[x][y]=false;
      num_obs[x][y]=0;
      num_clear[x][y]=0;
    }
  }

  // build height map
  for (unsigned i = 0; i < npoints; ++i) {
    // TODO: convert position of points from camera to local coordinate
    // - camera coordinate (x,y,z): scan->points[i].x, scan->points[i].y, scan->points[i].z
    // - local coordinate (x,y,z): local_x, local_y, local_z
    double local_x = scan->points[i].x; // currently, wrong transform. you need to fix this!
    double local_y = scan->points[i].y; // currently, wrong transform. you need to fix this!
    double local_z = scan->points[i].z; // currently, wrong transform. you need to fix this!

    int x = ((grid_dim_/2)+(local_x)/m_per_cell_);
    int y = ((grid_dim_/2)+(local_y)/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_) {
      if (!init[x][y]) {
        min[x][y] = local_z;
        max[x][y] = local_z;
        num_obs[x][y] = 0;
        num_clear[x][y] = 0;
        init[x][y] = true;
      } else {
        min[x][y] = MIN(min[x][y], local_z);
        max[x][y] = MAX(max[x][y], local_z);
      }
    }
  }

  // calculate height (max_z - min_z) of each grid in each cell
  for (unsigned i = 0; i < npoints; ++i) {
    // TODO: convert position of points from camera to local coordinate
    // - camera coordinate (x,y,z): scan->points[i].x, scan->points[i].y, scan->points[i].z
    // - local coordinate (x,y,z): local_x, local_y, local_z
    double local_x = scan->points[i].x; // currently, wrong transform. you need to fix this!
    double local_y = scan->points[i].y; // currently, wrong transform. you need to fix this!
    
    int x = ((grid_dim_/2)+(local_x)/m_per_cell_);
    int y = ((grid_dim_/2)+(local_y)/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]) {
      if ((max[x][y] - min[x][y] > height_diff_threshold_) ) {  
        num_obs[x][y]++;
      } else {
        num_clear[x][y]++;
      }
    }
  }

  // create clouds from grid
  double grid_offset=grid_dim_/2.0*m_per_cell_;
  for (int x = 0; x < grid_dim_; x++) {
    for (int y = 0; y < grid_dim_; y++) {
      if (num_obs[x][y]>0) {

        obstacle_cloud_.points[obs_count].x = -grid_offset + (x*m_per_cell_+m_per_cell_/2.0);
        obstacle_cloud_.points[obs_count].y = -grid_offset + (y*m_per_cell_+m_per_cell_/2.0);
        obstacle_cloud_.points[obs_count].z = height_diff_threshold_;
        //obstacle_cloud_.channels[0].values[obs_count] = (float) 255.0;
        obs_count++;
      }
      if (num_clear[x][y]>0) {
        clear_cloud_.points[empty_count].x = -grid_offset + (x*m_per_cell_+m_per_cell_/2.0);
        clear_cloud_.points[empty_count].y = -grid_offset + (y*m_per_cell_+m_per_cell_/2.0);
        clear_cloud_.points[empty_count].z = height_diff_threshold_;
        //clear_cloud_.channels[0].values[empty_count] = (float) 255.0;
        empty_count++;
      }
    }
  }
}

/** point cloud input callback */
void HeightMap::processData(const VPointCloud::ConstPtr &scan)
{
  if ((obstacle_publisher_.getNumSubscribers() == 0)
      && (clear_publisher_.getNumSubscribers() == 0))
    return;
  
  // pass along original time stamp and frame ID
  obstacle_cloud_.header.stamp = scan->header.stamp;
  obstacle_cloud_.header.frame_id = "camera_link"; // scan->header.frame_id

  // pass along original time stamp and frame ID
  clear_cloud_.header.stamp = scan->header.stamp;
  clear_cloud_.header.frame_id = "camera_link"; // scan->header.frame_id

  // set the exact point cloud size -- the vectors should already have
  // enough space
  size_t npoints = scan->points.size();
  obstacle_cloud_.points.resize(npoints);
  //obstacle_cloud_.channels[0].values.resize(npoints);

  clear_cloud_.points.resize(npoints);
  //clear_cloud_.channels[0].values.resize(npoints);

  size_t obs_count=0;
  size_t empty_count=0;
  // either return full point cloud or a discretized version
  if (full_clouds_)
    constructFullClouds(scan,npoints,obs_count, empty_count);
  else
    constructGridClouds(scan,npoints,obs_count, empty_count);
  
  obstacle_cloud_.points.resize(obs_count);
  //obstacle_cloud_.channels[0].values.resize(obs_count);

  clear_cloud_.points.resize(empty_count);
  //clear_cloud_.channels[0].values.resize(empty_count);
  
  if (obstacle_publisher_.getNumSubscribers() > 0)
    obstacle_publisher_.publish(obstacle_cloud_);

  if (clear_publisher_.getNumSubscribers() > 0)
    clear_publisher_.publish(clear_cloud_);
}

} // namespace local_costmap_generator
