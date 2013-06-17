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

  ROS_INFO("Pinging device...");

  imu.setTimeout(posix_time::seconds(5));

  if (imu.ping()) ROS_INFO("Device responds.");
  else {

  	  string msg = "...";

  	  while(msg!="") {

  		  msg = imu.getLastError();
  		  ROS_ERROR("Ping failed: %s",msg.c_str());

  	  }

  }

  ROS_INFO("Checking device status...");

  if (imu.devStatus()) ROS_INFO("Device status checked.");
  else {

	  string msg;

	  do {

		  msg = imu.getLastError();
		  if (msg!="") ROS_ERROR("Device status: %s",msg.c_str());

	  } while (msg!="");

	  imu.closePort();
	  return 0;

  }

  ROS_INFO("Running selftest (can take up to 5 seconds).");

  if (imu.selfTest()) ROS_INFO("Selftest passed.");
  else {

	  string msg;

	  do {

		  msg = imu.getLastError();
		  if (msg!="") ROS_ERROR("Selftest failed: %s",msg.c_str());

	  } while(msg!="");

	  imu.closePort();
	  return 0;

  }


  ROS_INFO("Finished");

  imu.closePort();

  return 0;

}
