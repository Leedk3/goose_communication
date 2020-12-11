/*
 * route_planner.h
 *
 *  Created on: Oct 25, 2020
 *      Author: Daegyu Lee
 */
#ifndef GOOSE_COMM_H
#define GOOSE_COMM_H

// headers in ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

// headers in STL
#include <memory>
#include <cmath>
#include <type_traits>
#include <stdio.h>
#include <float.h>
#include <vector>
#include <queue>
#include <deque>
#include <algorithm>
#include <unordered_map>
#include <bits/stdc++.h>
#include <mutex>
#include <thread>

#include <pugixml.hpp>

//User defined messages
#include "goose_comm_msgs/node_link.h"
#include "goose_comm_msgs/sensor_data.h"

#include "goose_comm/UTM.h"

#include "goose_comm_msgs/OsmParcer.h"
#include "goose_comm_msgs/Way.h"
#include "goose_comm_msgs/Node.h"

// PCL Libraries
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

class GooseComm
{
public:
  GooseComm(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
  ~GooseComm();

  void OsmParcing();
  void OsmToRosMsg();  
  void run();
  void CallbackOdometry(const nav_msgs::OdometryConstPtr& msg);
  
private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher pubNodeLink;
  ros::Publisher pubSensorData;
  ros::Publisher pubOsmParcer;
  ros::Publisher pubPluxityMapPoints;
  ros::Subscriber SubOdometry;
  
  std::string m_OsmFileName, m_FullFilePath;
  
  nav_msgs::Odometry m_Odometry;
  goose_comm_msgs::node_link m_NodeLink;
  goose_comm_msgs::sensor_data m_SensorData;
  goose_comm_msgs::OsmParcer m_OsmParcer;

  bool bParcingComplete;

};

#endif  // GOOSE_COMM_H
