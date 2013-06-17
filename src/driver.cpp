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

  comm_mode = -1;

  //cout<<"class created"<<endl;

}

IMU::~IMU() {

  //cout<<"class destroyed"<<endl;

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



bool IMU::ping() {

	return sendNoDataCmd(CMD_SET_BASIC, CMD_BASIC_PING);

}

bool IMU::pollAHRS() {

	tbyte_array data;

	data.push_back(sync1);
	data.push_back(sync2);
	data.push_back(CMD_SET_3DM); // desc set
	data.push_back(0x04); // length
	data.push_back(0x04);
	data.push_back(CMD_3DM_POLL_AHRS);
	data.push_back(0x1); // suppress ACK
	data.push_back(0x0);

	crc(data);
	write(data);

	tbyte_array recv;

	size_t n = 48; // TODO this is really stupid... there must be some parsing etc....

	recv = read(n);

	if (!crcCheck(recv)) return false;

	//if (!checkACK(recv,CMD_SET_3DM,CMD_3DM_POLL_AHRS)) return false;

	// quaternion 0x0A, field length 18, MSB first

	//quat.time = posix_time::microsec_clock::local_time();
	struct timespec curtime;
	clock_gettime(CLOCK_REALTIME, &curtime);
	ahrs_data_.time =  (uint64_t)(curtime.tv_sec) * 1000000000 + (uint64_t)(curtime.tv_nsec);

	if (recv[4] != 0x0E || recv[5] != 0x04) {

		errMsg("AHRS: Wrong msg format (0x04).");
		return false;

	}

	ahrs_data_.ax = extractFloat(&recv[6]); // 0x04
	ahrs_data_.ay = extractFloat(&recv[6+4]);
	ahrs_data_.az = extractFloat(&recv[6+8]);

	if (recv[18] != 0x0E || recv[19] != 0x05) {

		errMsg("AHRS: Wrong msg format (0x05).");
		return false;

	}

	ahrs_data_.gx = extractFloat(&recv[20]); // 0x05
	ahrs_data_.gy = extractFloat(&recv[20+4]);
	ahrs_data_.gz = extractFloat(&recv[20+8]);

	if (recv[32] != 0x0E || recv[33] != 0x0C) {

		errMsg("AHRS: Wrong msg format (0x0C).");
		return false;

	}

	ahrs_data_.r = extractFloat(&recv[34]); // 0x0C
	ahrs_data_.p = extractFloat(&recv[34+4]);
	ahrs_data_.y = extractFloat(&recv[34+8]);

	/*quat.q0 = extractFloat(&recv[6]);
	quat.q1 = extractFloat(&recv[6+4]);
	quat.q2 = extractFloat(&recv[6+8]);
	quat.q3 = extractFloat(&recv[6+12]);*/

	//cout << quat.q0 << " " << quat.q1 << " " << quat.q2 << " " << quat.q3 << endl;

	return true;

}

tahrs IMU::getAHRS() {

	return ahrs_data_;

}

float IMU::extractFloat(char* addr) {

  float tmp;

  *((unsigned char*)(&tmp) + 3) = *(addr);
  *((unsigned char*)(&tmp) + 2) = *(addr+1);
  *((unsigned char*)(&tmp) + 1) = *(addr+2);
  *((unsigned char*)(&tmp)) = *(addr+3);

  return tmp;
}

bool IMU::setAHRSMsgFormat() { // TODO add some parameters...

	tbyte_array data;

	data.push_back(sync1);
	data.push_back(sync2);
	data.push_back(CMD_SET_3DM); // desc set
	data.push_back(0x0D); // length
	data.push_back(0x0D);
	data.push_back(CMD_3DM_AHRS_MSG_FORMAT);

	data.push_back(FUN_USE_NEW);
	data.push_back(0x03); // desc count

	data.push_back(0x04); // accelerometer vector
	data.push_back(0x0);
	data.push_back(0x5); // 20 Hz (100 Hz / 5)

	data.push_back(0x05); // gyro vector
	data.push_back(0x0);
	data.push_back(0x5); // 20 Hz (100 Hz / 5)

	data.push_back(0x0C); // euler angles
	data.push_back(0x0);
	data.push_back(0x5); // 20 Hz (100 Hz / 5)

	crc(data);

	write(data);

	tbyte_array recv;

	size_t n = 10;

	recv = read(n);

	if (!crcCheck(recv)) return false;

	if (!checkACK(recv,CMD_SET_3DM,CMD_3DM_AHRS_MSG_FORMAT)) return false;

	return true;

}

bool IMU::initKalmanFilter(uint32_t decl) {

	tbyte_array data;

	data.push_back(sync1);
	data.push_back(sync2);
	data.push_back(CMD_SET_NAVFILTER); // desc set
	data.push_back(0x06); // length
	data.push_back(0x06);
	data.push_back(CMD_NAV_SET_INIT_FROM_AHRS);

	data.push_back((decl>>16)&0xff); // MSB
	data.push_back((decl>>8)&0xff);
	data.push_back((decl>>4)&0xff);
	data.push_back((decl)&0xff); // LSB

	crc(data);

	write(data);

	tbyte_array recv;

	size_t n = 10;

	recv = read(n);

	if (!crcCheck(recv)) return false;

	if (!checkACK(recv,CMD_SET_NAVFILTER,CMD_NAV_SET_INIT_FROM_AHRS)) return false;

	return true;

}

bool IMU::resume() {

	return sendNoDataCmd(CMD_SET_BASIC, CMD_BASIC_RESUME);

}


bool IMU::setToIdle() {

	return sendNoDataCmd(CMD_SET_BASIC, CMD_BASIC_SET_TO_IDLE);

}

bool IMU::sendNoDataCmd(uint8_t cmd_set, uint8_t cmd) {

	tbyte_array data;

	data.push_back(sync1);
	data.push_back(sync2);
	data.push_back(cmd_set); // desc set
	data.push_back(0x02); // length
	data.push_back(0x02);
	data.push_back(cmd);

	crc(data);

	write(data);

	tbyte_array recv;

	size_t n = 10;

	recv = read(n);

	if (!crcCheck(recv)) return false;

	if (!checkACK(recv,cmd_set,cmd)) return false;

	return true;

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
	data.push_back(CMD_BASIC_DEV_BUILTIN_TEST);

	crc(data);
	write(data);

	tbyte_array recv;
	size_t n = 16;

	recv = read(n);

	if (!crcCheck(recv)) {

		setTimeout(timeout_orig);
		return false;

	}

	setTimeout(timeout_orig);

	if (!checkACK(recv,CMD_SET_BASIC, CMD_BASIC_DEV_BUILTIN_TEST)) return false;

	if (recv[10]==0 && recv[11]==0 && recv[12]==0 && recv[13]==0) return true;
	else {

		if (recv[10] & 0x1) errMsg("AP-1: I2C Hardware Error.");
		if (recv[10] & 0x2) errMsg("AP-1: I2C EEPROM Error.");

		if (recv[11] & 0x1) errMsg("AHRS: Communication Error.");

		if (recv[12] & 0x1) errMsg("GPS: Communication Error.");
		if (recv[12] & 0x2) errMsg("GPS: 1PPS Signal Error.");
		if (recv[12] & 0x4) errMsg("GPS: 1PPS Inhibit Error.");
		if (recv[12] & 0x8) errMsg("GPS: Power Control Error.");

		return false;

	}

}


bool IMU::devStatus() {

	tbyte_array data;

	data.push_back(sync1);
	data.push_back(sync2);
	data.push_back(CMD_SET_3DM);
	data.push_back(0x05);
	data.push_back(0x05);
	data.push_back(CMD_3DM_DEV_STATUS);
	//data.push_back(((uint16_t)MODEL_ID>>2) & 0xff);
	//data.push_back((uint16_t)MODEL_ID & 0xff);
	data.push_back(0x18);
	data.push_back(0x54);
	data.push_back(0x01); // basic status


	crc(data);
	write(data);

	tbyte_array recv;
	size_t n = 27;

	recv = read(n);

	if (!crcCheck(recv)) {

		return false;

	}

	if (!checkACK(recv,CMD_SET_3DM, CMD_3DM_DEV_STATUS)) return false;

	//if (((uint8_t)recv[10] != ((MODEL_ID>>2)&0xff)) || ((uint8_t)recv[11] != (MODEL_ID & 0xff))) {
	if (((uint8_t)recv[10] != 0x18) || ((uint8_t)recv[11] != 0x54)) {

		errMsg("Wrong model number.");
		return false;

	}

	if (recv[13] != COMM_MODE_MIP) {

		errMsg("Not in MIP mode.");
		return false;

	}

	return true;

}

bool IMU::setStream(uint8_t stream, bool state) {

	tbyte_array data;

	data.push_back(sync1);
	data.push_back(sync2);
	data.push_back(CMD_SET_3DM);
	data.push_back(0x05);
	data.push_back(0x05);
	data.push_back(CMD_3DM_STREAM_STATE);
	data.push_back(0x1);
	data.push_back(stream);
	if (state) data.push_back(0x01);
	else data.push_back(0x0);


	crc(data);
	write(data);

	tbyte_array recv;
	size_t n = 10;

	recv = read(n);

	if (!crcCheck(recv)) {

		return false;

	}

	if (!checkACK(recv,CMD_SET_3DM, CMD_3DM_STREAM_STATE)) return false;

	return true;

}

bool IMU::disAllStreams() {

	bool ret = true;

	if (!setStream(0x01,false)) ret = false; // AHRS
	if (!setStream(0x02,false)) ret = false; // GPS
	if (!setStream(0x03,false)) ret = false; // NAV

	return ret;

}

void IMU::errMsg(std::string msg) {

	error_desc.push_back(msg);

}

bool IMU::checkACK(tbyte_array& arr, uint8_t cmd_set, uint8_t cmd) {

	if (arr.size() < 8) {

		errMsg("Too short reply.");
		return false;

	}

	if (arr[0] != sync1 || arr[1] != sync2) {

		errMsg("Strange synchronization bytes.");
		return false;

	}

	if (arr[2] != cmd_set) {

		errMsg("Wrong desc set in reply.");
		return false;

	}

	if (arr[6] != cmd) {

		errMsg("Wrong command echo.");
		return false;

	}

	if (arr[7] != 0x0) {

		errMsg("NACK.");
		return false;

	}

	return true;

}

string IMU::getLastError() {

	if (error_desc.size() > 0) {

		string tmp;

		tmp = error_desc.back();
		error_desc.pop_back();
		return tmp;

	} else return "";

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
	else {

		errMsg("Bad CRC.");
		return false;

	}

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
