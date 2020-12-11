/*
 * route_planner.cpp
 *
 *  Created on: Oct 25, 2020
 *      Author: Daegyu Lee
 */
#include "goose_comm/goose_comm.h"

#include <string>
#include <assert.h>

using namespace usrg_utm;
#define INF 9999999
// Constructor
GooseComm::GooseComm(ros::NodeHandle &nh, ros::NodeHandle &private_nh) : nh_(nh), private_nh_(private_nh), bParcingComplete(false)
                                                            
{
  private_nh_.param("osm_name", m_OsmFileName, std::string("kaist_new.osm"));


  goose_comm_msgs::node_link m_NodeLink;
  goose_comm_msgs::sensor_data m_SensorData;

  pubNodeLink = nh_.advertise<goose_comm_msgs::node_link>("/goose_comm/json_node_link", 1,true);
  pubOsmParcer = nh_.advertise<goose_comm_msgs::OsmParcer>("/goose_comm/osm_parcer", 1,true);
  pubPluxityMapPoints = nh_.advertise<sensor_msgs::PointCloud2>("/goose_comm/pluxity_full_map", 1,true);
  

  SubOdometry = nh_.subscribe("/lego_odom_ndt", 1, &GooseComm::CallbackOdometry, this);
  
  OsmParcing();
}

GooseComm::~GooseComm(){}

void GooseComm::OsmParcing()
{
  ROS_INFO("opening the osm map file..");
  //Open the file 
	std::string package_path  = ros::package::getPath("goose_comm");
	m_FullFilePath = (package_path + "/osm/" + m_OsmFileName).c_str();
  pugi::xml_document doc;  
  pugi::xml_parse_result parcer = doc.load_file(m_FullFilePath.c_str());  
  
  //file opening result
  if (!parcer)
  {
    ROS_ERROR("Invalid file path.. ");
    std::cout << "Parse error: " << parcer.description()
        << ", character pos= " << parcer.offset << std::endl; 
    std::cout << "Tried to open .. \n"<< m_FullFilePath.c_str() << std::endl;
  }
  else
  {
    std::cout << "Parse result: " << parcer.description()
    << ", character pos= " << parcer.offset << std::endl;
    std::cout << "Tried to open .. \n"<< m_FullFilePath.c_str() << std::endl;
    ROS_INFO("Vaild file!");
  }

  sensor_msgs::NavSatFix origin_llh;
  origin_llh.latitude = 36.3660857;
  origin_llh.longitude = 127.3636824;
  usrg_utm::UtmProjector m_UtmProjector(origin_llh);

  //Node data
  for (pugi::xml_node node: doc.child("osm").children("node"))
  {
    // std::cout << node.attribute("id").value() << ", lat: " << node.attribute("lat").value() << ", lon: " << node.attribute("lon").value()<< std::endl;

    goose_comm_msgs::Node nodeTmp;
    sensor_msgs::NavSatFix gpsTmp;
    nodeTmp.id = node.attribute("id").as_int();
    nodeTmp.lat = node.attribute("lat").as_double();
    nodeTmp.lon = node.attribute("lon").as_double();
    gpsTmp.latitude = node.attribute("lat").as_double();
    gpsTmp.longitude = node.attribute("lon").as_double();
    geometry_msgs::Pose2D pose2DTmp = m_UtmProjector.forward(gpsTmp);

    double yaw_bais_deg = -123.8;
    double yaw_bias = yaw_bais_deg * M_PI / 180;
    nodeTmp.x = pose2DTmp.x * cos(yaw_bias) - pose2DTmp.y * sin(yaw_bias);
    nodeTmp.y = pose2DTmp.x * sin(yaw_bias) + pose2DTmp.y * cos(yaw_bias);
    
    // nodeTmp.x = pose2DTmp.x;
    // nodeTmp.y = pose2DTmp.y;  
    //Tags 
    for(pugi::xml_node tag : node.children("tag"))
    {
      std::string highway = "highway";
      if(tag.attribute("k").as_string() == highway)
      {
        // std::cout << "yaw: " << tag.attribute("v").as_double() << std::endl;
        nodeTmp.highway = tag.attribute("v").as_string();
      }
      std::string traffic_signals = "traffic_signals";
      if(tag.attribute("k").as_string() == traffic_signals)
      {
        // std::cout << "speed: " << tag.attribute("v").as_double() << std::endl;
        nodeTmp.traffic_signals = tag.attribute("v").as_string();
      }
    }  

    m_OsmParcer.nodes.push_back(nodeTmp);
  }
  //Way data
  goose_comm_msgs::OsmParcer WayParcer;
  for (pugi::xml_node way: doc.child("osm").children("way"))
  {
    //Way id
    // std::cout << "---------------" << std::endl;
    // std::cout << "way id: " << way.attribute("id").value() << std::endl;
    goose_comm_msgs::Way wayTmp;
    wayTmp.id = way.attribute("id").as_int();
    //attribute of way data : Node IDs
    for(pugi::xml_node way_data : way.children("nd"))
    {
      // std::cout << "way id: "<< way.attribute("id").value() <<", ref: " << way_data.attribute("ref").value() << std::endl;
      goose_comm_msgs::Node nodeRef; //Node Ids in the each way
      nodeRef.id = way_data.attribute("ref").as_int();
      for (auto node: m_OsmParcer.nodes)
      {
        if(node.id == nodeRef.id)
        {
          nodeRef = node;
        }
      }
      wayTmp.nodes.push_back(nodeRef);
    }
    //Tags 
    for(pugi::xml_node tag : way.children("tag"))
    {
      std::string access = "access";
      if(tag.attribute("k").as_string() == access)
      {
        // std::cout << "access: " << tag.attribute("v").as_string() << std::endl;
        wayTmp.access = tag.attribute("v").as_string();
      }
      std::string bicycle = "bicycle";
      if(tag.attribute("k").as_string() == bicycle)
      {
        // std::cout << "bicycle: " << tag.attribute("v").as_string() << std::endl;
        wayTmp.bicycle = tag.attribute("v").as_string();
      }
      std::string foot = "foot";
      if(tag.attribute("k").as_string() == foot)
      {
        // std::cout << "foot: " << tag.attribute("v").as_string() << std::endl;
        wayTmp.foot = tag.attribute("v").as_string();
      }
      std::string highway = "highway";
      if(tag.attribute("k").as_string() == highway)
      {
        // std::cout << "highway: " << tag.attribute("v").as_string() << std::endl;
        wayTmp.highway = tag.attribute("v").as_string();
      }
      std::string maxspeed = "maxspeed";
      if(tag.attribute("k").as_string() == maxspeed)
      {
        // std::cout << "maxspeed: " << tag.attribute("v").as_string() << std::endl;
        wayTmp.maxspeed = tag.attribute("v").as_string();
      }
      std::string name = "name";
      if(tag.attribute("k").as_string() == name)
      {
        // std::cout << "name: " << tag.attribute("v").as_string() << std::endl;
        wayTmp.name = tag.attribute("v").as_string();
      }
      std::string oneway = "oneway";
      if(tag.attribute("k").as_string() == oneway)
      {
        // std::cout << "oneway: " << tag.attribute("v").as_string() << std::endl;
        wayTmp.oneway = tag.attribute("v").as_string();
      }
      std::string sidewalk = "sidewalk";
      if(tag.attribute("k").as_string() == sidewalk)
      {
        // std::cout << "sidewalk: " << tag.attribute("v").as_string() << std::endl;
        wayTmp.sidewalk = tag.attribute("v").as_string();
      }  
      std::string width = "width";
      if(tag.attribute("k").as_string() == width)
      {
        // std::cout << "width: " << tag.attribute("v").as_string() << std::endl;
        wayTmp.width = tag.attribute("v").as_string();
      }  
      std::string Slope = "Slope";
      if(tag.attribute("k").as_string() == Slope)
      {
        // std::cout << "Slope: " << tag.attribute("v").as_string() << std::endl;
        wayTmp.Slope = tag.attribute("v").as_string();
      }  
      std::string Left_curb = "Left-curb";
      if(tag.attribute("k").as_string() == Left_curb)
      {
        // std::cout << "Left_curb: " << tag.attribute("v").as_string() << std::endl;
        wayTmp.Left = tag.attribute("v").as_string();
      }  
      std::string Right_curb = "Right-curb";
      if(tag.attribute("k").as_string() == Right_curb)
      {
        // std::cout << "Right_curb: " << tag.attribute("v").as_string() << std::endl;
        wayTmp.Right = tag.attribute("v").as_string();
      }                                                      
      std::string Pavement = "Pavement";
      if(tag.attribute("k").as_string() == Pavement)
      {
        // std::cout << "Pavement: " << tag.attribute("v").as_string() << std::endl;
        wayTmp.Pavement = tag.attribute("v").as_string();
      }         
    }
    m_OsmParcer.ways.push_back(wayTmp);  
  }


  // ROS_INFO("Converting OSM file to the ros system is completed!");
  // std::cout << "------------------------------------"<< std::endl;
  // std::cout << "node size: "<< m_OsmParcer.nodes.size() << std::endl;
  // std::cout << "way size: "<< m_OsmParcer.ways.size() << "\n" <<std::endl;

  bParcingComplete = true;
}

void GooseComm::run()
{
  m_OsmParcer.header.frame_id = "map";
  m_OsmParcer.header.stamp = ros::Time::now();
  pubOsmParcer.publish(m_OsmParcer);
  
  OsmToRosMsg();
}  

void GooseComm::CallbackOdometry(const nav_msgs::OdometryConstPtr& msg)
{
  m_Odometry = *msg;

  double minimumDistance = INF; 
  int closestNodeId = -1;
  int closestWayId = -1;
  int nodeIndex = 0;
  int wayIndex = 0;
  int closestNodeIndex =0;
  int closestWayIndex =0;


  if(bParcingComplete)
  {
    for(auto node : m_OsmParcer.nodes)
    {
      double distance2node = sqrt(pow(m_Odometry.pose.pose.position.x - node.x, 2) + 
                                  pow(m_Odometry.pose.pose.position.y - node.y, 2));
      if(distance2node < minimumDistance)
      {
        minimumDistance = distance2node;
        closestNodeId = node.id;
        closestNodeIndex = nodeIndex;
      }
      nodeIndex ++;
    }

    for(auto way: m_OsmParcer.ways)
    {
      for(auto nodeInWay : way.nodes)
      {
        if(nodeInWay.id == closestNodeId)
        {
          closestWayId = way.id;
          closestWayIndex = wayIndex;           
          break;
        }
      }
      wayIndex ++;
    }

    std::cout << "-----------CURRENT POSE INFO-----------" << std::endl;
    std::cout << "Full node size : " << m_OsmParcer.nodes.size() << ", way size" << m_OsmParcer.ways.size() << std::endl;
    // std::cout << "nodeIndex: " << closestNodeIndex << ", wayIndex" << closestWayIndex << std::endl;
    std::cout << "closest node info : " << m_OsmParcer.nodes[closestNodeIndex] << ", distance: "<< minimumDistance << std::endl;
    std::cout << "closest way info : " << m_OsmParcer.ways[closestWayIndex] << "\n" << std::endl; 

    goose_comm_msgs::node_link node_link_msg;    
    node_link_msg.access = m_OsmParcer.ways[closestWayIndex].access;
    node_link_msg.bicycle = m_OsmParcer.ways[closestWayIndex].bicycle;
    node_link_msg.foot = m_OsmParcer.ways[closestWayIndex].foot;
    node_link_msg.highway = m_OsmParcer.ways[closestWayIndex].highway;
    node_link_msg.maxspeed = m_OsmParcer.ways[closestWayIndex].maxspeed;
    node_link_msg.name = m_OsmParcer.ways[closestWayIndex].name;
    node_link_msg.oneway = m_OsmParcer.ways[closestWayIndex].oneway;
    node_link_msg.sidewalk = m_OsmParcer.ways[closestWayIndex].sidewalk;
    node_link_msg.width = m_OsmParcer.ways[closestWayIndex].width;
    node_link_msg.Slope = m_OsmParcer.ways[closestWayIndex].Slope;
    node_link_msg.Left = m_OsmParcer.ways[closestWayIndex].Left;
    node_link_msg.Right = m_OsmParcer.ways[closestWayIndex].Right;
    node_link_msg.Pavement = m_OsmParcer.ways[closestWayIndex].Pavement;

    pubNodeLink.publish(node_link_msg);
  }
  else
  {
    ROS_WARN("Map was not loaded yet. please wait for map loading...");
  }  

}


void GooseComm::OsmToRosMsg()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  int count = 0;
  for(auto node: m_OsmParcer.nodes)
  {
    pcl::PointXYZI current_point;
    current_point.x = node.x;
    current_point.y = node.y;
    current_point.intensity = count;

    cloud_ptr->points.push_back(current_point);
    count ++;     
  }

  sensor_msgs::PointCloud2 PluxityMapCloudMsg;
  pcl::toROSMsg(*cloud_ptr, PluxityMapCloudMsg);
  PluxityMapCloudMsg.header.frame_id = "map";
  PluxityMapCloudMsg.header.stamp = ros::Time::now();
  pubPluxityMapPoints.publish(PluxityMapCloudMsg);

}