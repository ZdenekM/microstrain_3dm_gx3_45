#include "microstrain_3dm_gx3_45/driver.h"

#include <algorithm>
#include <iostream>
#include <boost/bind.hpp>

using namespace microstrain_3dm_gx3_45;
using namespace std;
using namespace boost;



IMU::IMU() : io(), serial(io), timer(io), timeout(posix_time::seconds(0)) {

  sync1 = 0x75;
  sync2 = 0x65;

  cout<<"class created"<<endl;

}

IMU::~IMU() {

  cout<<"class destroyed"<<endl;

}

bool IMU::openPort(string port, unsigned int baud_rate, boost::asio::serial_port_base::parity opt_parity,
    boost::asio::serial_port_base::character_size opt_csize,
    boost::asio::serial_port_base::flow_control opt_flow,
    boost::asio::serial_port_base::stop_bits opt_stop) {

	if(isOpen()) closePort();

	serial.open(port);
	serial.set_option(asio::serial_port_base::baud_rate(baud_rate));
	serial.set_option(opt_parity);
	serial.set_option(opt_csize);
	serial.set_option(opt_flow);
	serial.set_option(opt_stop);

	/*try {


	} catch(boost::system::system_error& e) {

		  cout<<"Error: "<<e.what()<<endl;
		  return false;

	  }*/

	return true;

}

bool IMU::isOpen() const {

	return serial.is_open();

}

void IMU::closePort() {

	if(isOpen()==false) return;
	serial.close();

}

void IMU::setTimeout(const posix_time::time_duration& t)
{
    timeout=t;
}

void IMU::write(const tbyte_array& data)
{
    asio::write(serial,asio::buffer(&data[0],data.size()));
}

void IMU::timeoutExpired(const boost::system::error_code& error)
{
     if(!error && result==resultInProgress) result=resultTimeoutExpired;
}

void IMU::readCompleted(const boost::system::error_code& error,
        const size_t bytesTransferred)
{
    if(!error)
    {
        result=resultSuccess;
        this->bytesTransferred=bytesTransferred;
        return;
    }

    if(error.value()==125) return; //Linux outputs error 125

    result=resultError;
}

tbyte_array IMU::read(size_t size)
{
    tbyte_array result(size,'\0');//Allocate a vector with the desired size
    read(&result[0],size);//Fill it with values
    return result;
}

void IMU::read(char *data, size_t size)
{
    if(readData.size()>0)//If there is some data from a previous read
    {
    	basic_istream<char> is(&readData);
        size_t toRead=min(readData.size(),size);//How many bytes to read?
        is.read(data,toRead);
        data+=toRead;
        size-=toRead;
        if(size==0) return;//If read data was enough, just return
    }

    setupParameters=ReadSetupParameters(data,size);
    performReadSetup(setupParameters);

    //For this code to work, there should always be a timeout, so the
    //request for no timeout is translated into a very long timeout
    if(timeout!=posix_time::seconds(0)) timer.expires_from_now(timeout);
    else timer.expires_from_now(posix_time::hours(100000));

    timer.async_wait(boost::bind(&IMU::timeoutExpired,this,
                asio::placeholders::error));

    result=resultInProgress;
    bytesTransferred=0;
    for(;;)
    {
        io.run_one();
        switch(result)
        {
        	case resultInProgress:

        		continue;

            case resultSuccess:
                timer.cancel();
                return;
            case resultTimeoutExpired:
                serial.cancel();
                throw(timeout_exception("Timeout expired"));
            case resultError:
                timer.cancel();
                serial.cancel();
                throw(boost::system::system_error(boost::system::error_code(),
                        "Error while reading"));
            //if resultInProgress remain in the loop
        }
    }
}

void IMU::performReadSetup(const ReadSetupParameters& param)
{
    if(param.fixedSize)
    {
        asio::async_read(serial,asio::buffer(param.data,param.size),boost::bind(
                &IMU::readCompleted,this,asio::placeholders::error,
                asio::placeholders::bytes_transferred));
    } else {
        asio::async_read_until(serial,readData,param.delim,boost::bind(
                &IMU::readCompleted,this,asio::placeholders::error,
                asio::placeholders::bytes_transferred));
    }
}

void IMU::test() {

	char c;
	std::string result;
	for(;;)
	{
		asio::read(serial,asio::buffer(&c,1));

		cout << result << " ";

	}

}

bool IMU::ping() {

	tbyte_array data;

	data.push_back(sync1);
	data.push_back(sync2);
	data.push_back(CMD_SET_BASIC); // desc set
	data.push_back(0x02); // length
	data.push_back(0x02);
	data.push_back(BASIC_PING);

	crc(data);

	write(data);

	tbyte_array recv;

	size_t n = 10;

	recv = read(n);

	if (!crcCheck(recv)) return false;

	if ( ((uint8_t)recv[2]==CMD_SET_BASIC) && ((uint8_t)recv[5]==DESC_ACK) && ((uint8_t)recv[6]==BASIC_PING) ) return true;
	else return false;

}

bool IMU::selfTest() {

	posix_time::time_duration timeout_orig = timeout;

	setTimeout(posix_time::seconds(6));

	tbyte_array data;

	data.push_back(sync1);
	data.push_back(sync2);
	data.push_back(CMD_SET_BASIC);
	data.push_back(0x02);
	data.push_back(0x02);
	data.push_back(BASIC_DEV_BUILTIN_TEST);

	crc(data);
	write(data);

	tbyte_array recv;
	size_t n = 16;

	recv = read(n);

	if (!crcCheck(recv)) {

		//cout << "crc failed" << endl;
		setTimeout(timeout_orig);
		return false;

	}

	if ((uint8_t)recv[6]!=BASIC_DEV_BUILTIN_TEST) {

		//cout << "error code" << endl;
		setTimeout(timeout_orig);
		return false;

	}


	setTimeout(timeout_orig);

	if (recv[10]==0 && recv[11]==0 && recv[12]==0 && recv[13]==0) return true;
	else {

		//TODO give some description of error

		//cout << static_cast<int>(recv[10]) << ", " << static_cast<int>(recv[11]) << ", " << static_cast<int>(recv[12]) << ", " << static_cast<int>(recv[13]) << endl;

		return false;

	}


}

bool IMU::crcCheck(tbyte_array& arr) {

	unsigned char b1=0;
	unsigned char b2=0;

	//cout << arr.size() << endl;

	for(unsigned int i=0; i<(arr.size()-2); i++)
	{
	 b1 += arr[i];
	 b2 += b1;
	}

	/*for(unsigned int i=0; i<(arr.size()); i++)
	cout << static_cast<int>(arr[i]) << " ";
	cout << endl;*/


	if (b1==(unsigned char)arr[arr.size()-2] && b2==(unsigned char)arr[arr.size()-1]) return true;
	else return false;

}

void IMU::crc(tbyte_array& arr) {

	char b1=0;
	char b2=0;

	for(unsigned int i=0; i<arr.size(); i++)
	{
	 b1 += arr[i];
	 b2 += b1;
	}

	arr.push_back(b1);
	arr.push_back(b2);

}
