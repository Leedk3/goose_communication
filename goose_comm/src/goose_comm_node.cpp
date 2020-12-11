#include "goose_comm/goose_comm.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goose_comm");

  ros::NodeHandle nh("~");
  ros::NodeHandle private_nh("~");

  GooseComm comm(nh, private_nh);
  ros::Rate loop_rate(2);
  while(ros::ok())
  {
    ros::spinOnce();
    comm.run();
    loop_rate.sleep();
  }

  return 0;
}
