#include "ros/ros.h"
#include "microstrain_3dm_gx3_45/driver.h"

using namespace microstrain_3dm_gx3_45;
using namespace std;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "imu3dmgx3");
  
  IMU imu;

  if (!imu.openPort("/dev/ttyACM0",115200)) {

	  ROS_ERROR("Failed to open port.");

  }

  ROS_INFO("Started.");

  //imu.test();

  ros::spin();

  return 0;

}
