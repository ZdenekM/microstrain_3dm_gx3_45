#include "ros/ros.h"
#include "microstrain_3dm_gx3_45/driver.h"
#include "microstrain_3dm_gx3_45/node.h"

using namespace microstrain_3dm_gx3_45;
using namespace std;
using namespace boost;
using namespace ros;

/*
 * TODOs
 * prepare object instead of this ugly code
 * add some services etc.
 */


imuNode::imuNode() : nh_priv_("~") {

	param::param<string>("~port",port_,"/dev/ttyACM0");
	param::param<int>("~baud_rate",baud_rate_,115200);
	param::param<int>("~declination",declination_,0);
	param::param<string>("~frame_id",frame_id_,"/imu");
	param::param<float>("~rate",rate_,10.0);

	param::param<bool>("~publish_pose",publish_pose_,true);
	param::param<bool>("~publish_imu",publish_imu_,true);

	param::param("linear_acceleration_stdev", linear_acceleration_stdev_, 0.098);
	param::param("orientation_stdev", orientation_stdev_, 0.035);
	param::param("angular_velocity_stdev", angular_velocity_stdev_, 0.012);

	started_ = false;
	inited_ = false;

	if (!imu_.openPort(port_,(unsigned int)baud_rate_)) {

		ROS_ERROR("Can't open port.");

	}

	imu_data_pub_ = nh_priv_.advertise<sensor_msgs::Imu>("imu/data", 100);
	if (publish_pose_) imu_pose_pub_ = nh_priv_.advertise<geometry_msgs::PoseStamped>("imu/pose", 100);

	service_reset_ = service_reset_ = nh_priv_.advertiseService("reset_kf", &imuNode::srvResetKF,this);

}

bool imuNode::srvResetKF(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {


	ROS_INFO("Resetting KF.");

	if (!imu_.setToIdle()) ROS_ERROR("%s",imu_.getLastError().c_str());
	if (!imu_.initKalmanFilter(declination_)) ROS_ERROR("%s",imu_.getLastError().c_str());
	if (!imu_.resume()) ROS_ERROR("%s",imu_.getLastError().c_str());

	return true;

}

bool imuNode::init() {

	started_ = false;

	imu_.setTimeout(posix_time::seconds(0.5));
	if (!imu_.ping()) {

		printErrMsgs("Pinging device");
		return false;

	}

	if (!imu_.setToIdle()) {

		printErrMsgs("Setting to idle");
		return false;

	}

	if (!imu_.devStatus()) {

		printErrMsgs("Checking status");
		return false;

	}

	if (!imu_.disAllStreams()) {

		printErrMsgs("Disabling all streams");
		return false;

	}

	if (!imu_.selfTest()) {

		printErrMsgs("Device self test");
		return false;
	}

	if (!imu_.setAHRSMsgFormat()) {

		printErrMsgs("Setting AHRS msg format");
		return false;

	}

	if (!imu_.initKalmanFilter((uint32_t)declination_)) {

		printErrMsgs("KF initialization");
		return false;

	}

	inited_ = true;
	return true;

}

bool imuNode::start() {

	if (!imu_.resume()) {

			printErrMsgs("Resuming");
			return false;

		}

	started_ = true;
	return true;

}

bool imuNode::stop() {

	if (!imu_.setToIdle()) {

		printErrMsgs("To idle");
		return false;

	}

	started_ = false;
	return true;

}

void imuNode::spin() {

	start();

	Rate r(rate_);

	geometry_msgs::PoseStamped ps;
	ps.header.frame_id = frame_id_;
	ps.pose.position.x = ps.pose.position.y = ps.pose.position.z = 0.0;

	sensor_msgs::Imu imu;
	imu.header.frame_id = frame_id_;

	double angular_velocity_covariance = angular_velocity_stdev_ * angular_velocity_stdev_;
	double orientation_covariance = orientation_stdev_ * orientation_stdev_;
	double linear_acceleration_covariance = linear_acceleration_stdev_ * linear_acceleration_stdev_;

	imu.linear_acceleration_covariance[0] = linear_acceleration_covariance;
	imu.linear_acceleration_covariance[4] = linear_acceleration_covariance;
	imu.linear_acceleration_covariance[8] = linear_acceleration_covariance;

	imu.angular_velocity_covariance[0] = angular_velocity_covariance;
	imu.angular_velocity_covariance[4] = angular_velocity_covariance;
	imu.angular_velocity_covariance[8] = angular_velocity_covariance;

	imu.orientation_covariance[0] = orientation_covariance;
	imu.orientation_covariance[4] = orientation_covariance;
	imu.orientation_covariance[8] = orientation_covariance;

	ROS_INFO("Start polling device.");

	while(ok()) {

		if (publish_imu_ || publish_pose_) {

			if (!imu_.pollAHRS()) {

				printErrMsgs("AHRS");

			}

		}

		tahrs q = imu_.getAHRS();

		if (publish_imu_ && imu_data_pub_.getNumSubscribers() > 0) {

			imu.header.stamp.fromNSec(q.time);

			imu.linear_acceleration.x = -q.ax;
			imu.linear_acceleration.y = q.ay;
			imu.linear_acceleration.z = -q.az;

			imu.angular_velocity.x = -q.gx;
			imu.angular_velocity.y = q.gy;
			imu.angular_velocity.z = -q.gz;

			float yaw = q.y;

			yaw+=M_PIl;
			if (yaw > M_PIl) yaw-=2*M_PIl;

			tf::quaternionTFToMsg(tf::createQuaternionFromRPY(-q.p, q.p, -yaw), imu.orientation);

			imu_data_pub_.publish(imu);

		}

		if (publish_pose_ && imu_pose_pub_.getNumSubscribers() > 0) {

			//ps.header.stamp.fromBoost(q.time);
			ps.header.stamp.fromNSec(q.time);

			float yaw = q.y;
			yaw+=M_PIl;
			if (yaw > M_PIl) yaw-=2*M_PIl;

			tf::quaternionTFToMsg(tf::createQuaternionFromRPY(-q.p, q.p, -yaw),ps.pose.orientation);

			imu_pose_pub_.publish(ps);

		}

		r.sleep();
		spinOnce();

	};

	stop();

}

void imuNode::printErrMsgs(string prefix) {

	string msg = "...";

	while(msg!="") {

	  msg = imu_.getLastError();
	  if (msg!="") ROS_ERROR("%s: %s",prefix.c_str(),msg.c_str());

  }

}

imuNode::~imuNode() {

	imu_.closePort();

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "imu3dmgx3");

  imuNode node;

  ROS_INFO("Initializing.");
  if (!node.init()) return 0;

  node.spin();


  ROS_INFO("Finished");


  return 0;

}
