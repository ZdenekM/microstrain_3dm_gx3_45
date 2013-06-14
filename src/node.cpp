#include "ros/ros.h"
#include "microstrain_3dm_gx3_45/driver.h"

using namespace microstrain_3dm_gx3_45;
using namespace std;
using namespace boost;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "imu3dmgx3");
  
  IMU imu;

  if (!imu.openPort("/dev/ttyACM0",115200)) {

	  ROS_ERROR("Failed to open port.");

  }

  ROS_INFO("Started.");

  imu.setTimeout(posix_time::seconds(5));

  if (imu.ping()) ROS_INFO("Device responds");
  else ROS_WARN("No reply");

  ROS_INFO("Finished");

  imu.closePort();

  return 0;

}
