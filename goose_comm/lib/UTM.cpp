#include "goose_comm/UTM.h"

using namespace usrg_utm;

UtmProjector::UtmProjector(sensor_msgs::NavSatFix& origin) : m_isInNorthernHemisphere(true), m_origin(origin)
{
  GeographicLib::UTMUPS::Forward(m_origin.latitude, m_origin.longitude, m_zone,
                                 m_isInNorthernHemisphere, m_utmOrigin.x, m_utmOrigin.y);

}

geometry_msgs::Pose2D UtmProjector::forward(const sensor_msgs::NavSatFix& gps){

  geometry_msgs::Pose2D utmXY;

  if(!m_isInNorthernHemisphere)
    ROS_WARN("Is it Southern Hemisphere?");

  try {
    GeographicLib::UTMUPS::Forward(gps.latitude, gps.longitude, m_zone, m_isInNorthernHemisphere, utmXY.x, utmXY.y);

    // printf("utm Data: \n x: %.9f ,y: %.9f \n" , utmXY.x, utmXY.y);
    // printf("origin Data: \n x: %.9f ,y: %.9f \n" , m_utmOrigin.x, m_utmOrigin.y);

    utmXY.x = utmXY.x - m_utmOrigin.x;
    utmXY.y = utmXY.y - m_utmOrigin.y;

  } catch (GeographicLib::GeographicErr& e) {
    ROS_WARN("GeographicLib FORWARD ERROR!");
  }

  return utmXY;
}

sensor_msgs::NavSatFix UtmProjector::reverse(const geometry_msgs::Pose2D& utm){

  sensor_msgs::NavSatFix GpsRaw;
  geometry_msgs::Pose2D utmXY; 

  if(!m_isInNorthernHemisphere)
    ROS_WARN("Is it Southern Hemisphere?");

  try { 
    utmXY.x = utm.x + m_utmOrigin.x;
    utmXY.y = utm.y + m_utmOrigin.y;
    GeographicLib::UTMUPS::Reverse(m_zone, m_isInNorthernHemisphere, utmXY.x, utmXY.y, GpsRaw.latitude, GpsRaw.longitude);
  } catch (GeographicLib::GeographicErr& e) {
    ROS_WARN("UTM Reverse Error");
  }

  return GpsRaw;
}
