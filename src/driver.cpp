#include "microstrain_3dm_gx3_45/driver.h"


using namespace microstrain_3dm_gx3_45;
using namespace std;
using namespace boost;


IMU::IMU() : io() {

  cout<<"class created"<<endl;

}

IMU::~IMU() {

  if (serial!=NULL) {

	  serial->close();
	  delete serial;

  }

  cout<<"class destroyed"<<endl;

}

bool IMU::openPort(string port, unsigned int baud_rate) {

	try {

		serial = new boost::asio::serial_port(io,port);
		serial->set_option(asio::serial_port_base::baud_rate(baud_rate));

	} catch(boost::system::system_error& e) {

		  cout<<"Error: "<<e.what()<<endl;
		  return false;

	  }

	return true;

}

void IMU::test() {

	char c;
	std::string result;
	for(;;)
	{
		asio::read(*serial,asio::buffer(&c,1));

		cout << result << " ";

	}

}
