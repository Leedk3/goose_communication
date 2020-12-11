#include <ros/ros.h>
#include <cmath>
// for using serial communication

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include <GeographicLib/UTMUPS.hpp>

#include "goose_comm/UTM.h"
#include "goose_comm_msgs/node_link.h"
#include "goose_comm_msgs/sensor_data.h"
#include "goose_comm_msgs/OsmParcer.h"
#include "goose_comm_msgs/Way.h"
#include "goose_comm_msgs/Node.h"

class GOOSE_SENSOR_DATA
{
    public:
        GOOSE_SENSOR_DATA(ros::NodeHandle& n);        
        ~GOOSE_SENSOR_DATA();

        void init();

        void CallbackVehState(const nav_msgs::Odometry& msg);        
        void CallbackOdometry(const nav_msgs::Odometry& msg);
        void CallbackGpsRaw(const sensor_msgs::NavSatFix& msg);
        void CallbackImuRaw(const sensor_msgs::Imu& msg);
        void CallbackCollision(const std_msgs::Bool& msg);
        void CallbackOperationMode(const std_msgs::Int32& msg);
        void CallbackTargetObject(const visualization_msgs::Marker& msg);
        void CallbackCrossTrackError(const std_msgs::Float64& msg);
        void CallbackHeadingError(const std_msgs::Float64& msg);

        nav_msgs::Odometry m_VehState;
        nav_msgs::Odometry m_VehOdometry;
        sensor_msgs::NavSatFix m_GpsRaw;
        sensor_msgs::Imu m_ImuRaw;
        std_msgs::Bool m_Collision;

        bool bVehState;
        bool bVehOdometry;
        bool bGpsRaw;
        bool bImuRaw;
        bool bCollision;
        
        ros::NodeHandle nh;
        
        ros::Subscriber subVehState;
        ros::Subscriber subVehOdometry;
        ros::Subscriber subGpsRaw;
        ros::Subscriber subImuRaw;
        ros::Subscriber subCollision;
        ros::Subscriber subOperationMode;
        ros::Subscriber subTargetObject;

        double m_origin_lat;
        double m_origin_lon;
        double m_utm2gps_yaw_bias;
        sensor_msgs::NavSatFix m_origin_llh;
        sensor_msgs::NavSatFix m_estimated_llh;
        double m_heading;
        int m_OperationMode;
        int m_RTO2KAIST;
        double m_TTC;
        double m_CountStuck;
        double m_currentTime;
        double m_prevTime;
        geometry_msgs::Pose2D m_projectionPrev;
        double m_GpsYawRaw;
        double m_HeadingError;
        double m_CrossTrackError;

};

GOOSE_SENSOR_DATA::GOOSE_SENSOR_DATA(ros::NodeHandle& n) : nh(n), bVehState(false), bVehOdometry(false), bGpsRaw(false), 
                                                   bImuRaw(false), bCollision(false)
{
    subVehState = nh.subscribe("/Ackermann/veh_state",10,&GOOSE_SENSOR_DATA::CallbackVehState, this);
    subVehOdometry = nh.subscribe("/Odometry/ekf_slam",10,&GOOSE_SENSOR_DATA::CallbackOdometry, this);
    subGpsRaw = nh.subscribe("/gps/fix",10, &GOOSE_SENSOR_DATA::CallbackGpsRaw, this);
    subImuRaw = nh.subscribe("/imu/data",10, &GOOSE_SENSOR_DATA::CallbackImuRaw, this);
    subCollision = nh.subscribe("/Bool/collision",10, &GOOSE_SENSOR_DATA::CallbackCollision, this);
    subOperationMode = nh.subscribe("/Int/OperationMode",10, &GOOSE_SENSOR_DATA::CallbackOperationMode, this);
    subTargetObject = nh.subscribe("/Marker/VisualServoing/TargetObject", 10, &GOOSE_SENSOR_DATA::CallbackTargetObject, this);

    ROS_DEBUG("GOOSE_SENSOR_DATA is created");
    init();
};

GOOSE_SENSOR_DATA::~GOOSE_SENSOR_DATA() 
{    
    ROS_INFO("GOOSE_SENSOR_DATA destructor.");
}

void GOOSE_SENSOR_DATA::init()
{
    m_origin_lat = 36.3660857; //KAIST_GATE
    m_origin_lon = 127.3636824; //KAIST_GATE
    m_utm2gps_yaw_bias = -123.8;

    m_origin_llh.latitude =  m_origin_lat;
    m_origin_llh.longitude = m_origin_lon;

}

void GOOSE_SENSOR_DATA::CallbackVehState(const nav_msgs::Odometry& msg)
{
    m_VehState = msg;
    bVehState = true;
}

void GOOSE_SENSOR_DATA::CallbackOperationMode(const std_msgs::Int32& msg)
{
    m_OperationMode = msg.data;
    if(msg.data == 4)
        m_RTO2KAIST = 0;
    else if(msg.data > 0 && msg.data < 4)
        m_RTO2KAIST = 1;
    else
    {
        m_RTO2KAIST = 2;
        ROS_ERROR("Operation Mode ERROR");
    }
}

void GOOSE_SENSOR_DATA::CallbackOdometry(const nav_msgs::Odometry& msg)
{
    m_VehOdometry = msg;
    bVehOdometry = true;
    
    // Use UTM
    geometry_msgs::Pose2D current_odom;

    double yaw_bias = m_utm2gps_yaw_bias * M_PI / 180; //deg -> rad

    double x_tmp = msg.pose.pose.position.x;
    double y_tmp = msg.pose.pose.position.y;

    current_odom.x = x_tmp * cos(yaw_bias) - y_tmp * sin(yaw_bias);
    current_odom.y = x_tmp * sin(yaw_bias) + y_tmp * cos(yaw_bias);

    usrg_utm::UtmProjector utm2gps(m_origin_llh);    
    m_estimated_llh = utm2gps.reverse(current_odom);

    tf::Quaternion q(msg.pose.pose.orientation.x, 
                     msg.pose.pose.orientation.y, 
                     msg.pose.pose.orientation.z, 
                     msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    m_heading =  yaw *  180 / M_PI + yaw_bias;
    ROS_INFO("lat: %f, lon: %f, heading: %f", m_estimated_llh.latitude, m_estimated_llh.longitude, m_heading);
}

void GOOSE_SENSOR_DATA::CallbackGpsRaw(const sensor_msgs::NavSatFix& msg)
{
    m_GpsRaw = msg;
    bGpsRaw = true;
}

void GOOSE_SENSOR_DATA::CallbackImuRaw(const sensor_msgs::Imu& msg)
{
    m_ImuRaw = msg;
    bImuRaw = true;
}
void GOOSE_SENSOR_DATA::CallbackCollision(const std_msgs::Bool& msg)
{   
    m_currentTime = ros::Time::now().toSec();
    m_Collision = msg;

    double dt = m_currentTime - m_prevTime;
    if(!bCollision)
    {
        m_prevTime = m_currentTime;    
        bCollision = true;
        return;
    }
    
    if(msg.data)
        m_CountStuck += dt;
    else
    {
        m_CountStuck = 0;
    }

    m_prevTime = m_currentTime;
}

void GOOSE_SENSOR_DATA::CallbackTargetObject(const visualization_msgs::Marker& msg)
{

    double distToTargetObject = sqrt(pow(msg.pose.position.x,2) + 
                                     pow(msg.pose.position.y,2));
    if(distToTargetObject == 0)
        m_TTC = 10;
    m_TTC =  distToTargetObject / (m_VehState.twist.twist.linear.x) ;

    if(m_TTC > 10)
        m_TTC = 10.1;
}

void GOOSE_SENSOR_DATA::CallbackCrossTrackError(const std_msgs::Float64& msg)
{
    m_CrossTrackError = msg.data;
}
void GOOSE_SENSOR_DATA::CallbackHeadingError(const std_msgs::Float64& msg)
{
    m_HeadingError = msg.data;
}

int main(int argc, char** argv)
{    
    // node name initialization
    ros::init(argc, argv, "goose_sensor_data_comm");

    // assign node GOOSE_SENSOR_DATA
    ros::NodeHandle nh_;
    ros::Rate loop_rate(5);

    GOOSE_SENSOR_DATA sensor_data(nh_);

    ros::Publisher pubSensorData = nh_.advertise<goose_comm_msgs::sensor_data>("/goose_comm/json_sensor_data", 1,true);
    
    goose_comm_msgs::sensor_data ToSend;

    while(ros::ok()){
        ToSend.speed = sensor_data.m_VehState.twist.twist.linear.x;
        ToSend.vehLat = (double)sensor_data.m_estimated_llh.latitude; 
        ToSend.vehLng = (double)sensor_data.m_estimated_llh.longitude; 
        ToSend.vehHeading = (double)sensor_data.m_heading;
        ToSend.XAcc = (double)sensor_data.m_ImuRaw.linear_acceleration.x; 
        ToSend.YAcc = (double)sensor_data.m_ImuRaw.linear_acceleration.y; 
        ToSend.ZAcc = (double)sensor_data.m_ImuRaw.linear_acceleration.z; 
        ToSend.XGyro = (double)sensor_data.m_ImuRaw.angular_velocity.x; 
        ToSend.YGyro = (double)sensor_data.m_ImuRaw.angular_velocity.y; 
        ToSend.ZGyro = (double)sensor_data.m_ImuRaw.angular_velocity.z; 
        ToSend.gpsTime = (double)sensor_data.m_GpsRaw.header.stamp.toSec(); 
        ToSend.gpsLat = (double)sensor_data.m_GpsRaw.latitude; 
        ToSend.gpsLng = (double)sensor_data.m_GpsRaw.longitude; 
        ToSend.gpsHeight = (double)sensor_data.m_GpsRaw.altitude;
        ToSend.gpsHeading =  (double)sensor_data.m_GpsYawRaw;
        ToSend.gpsStatus = (uint32_t)1; //ublox data output: -1. chech this. 
        ToSend.TTC = (float)sensor_data.m_TTC; 
        ToSend.collision = (bool)sensor_data.m_Collision.data; 
        ToSend.crossTrackErr = (double)sensor_data.m_CrossTrackError; //Calculate using stanley method. 
        ToSend.headingErr = (double)sensor_data.m_HeadingError; //calculate using stanley method. 
        ToSend.vehStuck = (float)sensor_data.m_CountStuck; //
        ToSend.battery = 25;
        loop_rate.sleep();
        // loop sampling, ros
        ros::spinOnce();
    }

    return 0;
}
