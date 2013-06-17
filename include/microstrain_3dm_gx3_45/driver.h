#ifndef MS_3DMGX3_45_HH
#define MS_3DMGX3_45_HH

#include <iostream>
#include <boost/asio/serial_port.hpp> 
#include <boost/asio.hpp>
#include <boost/utility.hpp>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

/*
 *
 * Serial communication based on following tutorial: http://www.webalice.it/fede.tft/serial_port/serial_port.html
 *
 *
 */

namespace microstrain_3dm_gx3_45 {

	typedef std::vector<char> tbyte_array;

	typedef struct {

		float ax;
		float ay;
		float az;

		float gx;
		float gy;
		float gz;

		float r;
		float p;
		float y;

		uint64_t time;

	} tahrs;


	class timeout_exception: public std::runtime_error
	{
	public:
		timeout_exception(const std::string& arg): runtime_error(arg) {}
	};


  class IMU : private boost::noncopyable
    {
    
    public:

	  enum cmd_set {

		  CMD_SET_BASIC = 0x01,
		  CMD_SET_3DM = 0x0C,
		  CMD_SET_NAVFILTER = 0x0D,
		  CMD_SET_SYSTEM = 0x7F

	  };

	  enum cmd_set_basic {

		  CMD_BASIC_PING = 0x01,
		  CMD_BASIC_SET_TO_IDLE = 0x02,
		  CMD_BASIC_GET_DEV_INFO = 0x03,
		  CMD_BASIC_GET_DEV_DESC_SETS = 0x04,
		  CMD_BASIC_DEV_BUILTIN_TEST = 0x05,
		  CMD_BASIC_RESUME = 0x06,
		  CMD_BASIC_RESET = 0x7E

	  };

	  enum cmd_set_3dm {

		  CMD_3DM_POLL_AHRS = 0x01,
		  CMD_3DM_DEV_STATUS = 0x64,
		  CMD_3DM_STREAM_STATE = 0x11,
		  CMD_3DM_AHRS_MSG_FORMAT = 0x08

	  };

	  enum cmd_set_nav {

		  CMD_NAV_SET_INIT_FROM_AHRS = 0x04

	  };

	  enum glob_descs {

		  DESC_ACK = 0xF1

	  };

	  enum comm_modes {

		  COMM_MODE_MIP = 0x01,
		  COMM_MODE_AHRS = 0x02,
		  COMM_MODE_GPS = 0x03

	  };

	  enum others {

		  MODEL_ID = 0x1854,
		  DATA = 0x80

	  };

	  enum functions {

		  FUN_USE_NEW = 0x01,
		  FUN_READ_CURRENT = 0x02,
		  FUN_SAVE_CURR_AS_STARTUP = 0x03,
		  FUN_LOAD_SAVE_STARTUP = 0x04,
		  FUN_RESET_TO_FACTORY_DEF = 0x05

	  };
    
      IMU();

      bool openPort(std::string port, unsigned int baud_rate, boost::asio::serial_port_base::parity opt_parity=
              boost::asio::serial_port_base::parity(
                  boost::asio::serial_port_base::parity::none),
          boost::asio::serial_port_base::character_size opt_csize=
              boost::asio::serial_port_base::character_size(8),
          boost::asio::serial_port_base::flow_control opt_flow=
              boost::asio::serial_port_base::flow_control(
                  boost::asio::serial_port_base::flow_control::none),
          boost::asio::serial_port_base::stop_bits opt_stop=
              boost::asio::serial_port_base::stop_bits(
                  boost::asio::serial_port_base::stop_bits::one));

      ~IMU();


      bool isOpen() const;

      void closePort();

      void setTimeout(const boost::posix_time::time_duration& t);

      bool ping();

      bool selfTest();

      bool devStatus();

      bool disAllStreams();

      std::string getLastError();

      bool setAHRSMsgFormat();

      bool setToIdle();

      bool resume();

      bool initKalmanFilter(uint32_t decl);

      bool pollAHRS();

      bool setStream(uint8_t stream, bool state);

      tahrs getAHRS();
    
    private:

    protected:

      tahrs ahrs_data_;

      void crc(tbyte_array& arr);
      bool crcCheck(tbyte_array& arr);

      int comm_mode;

      char sync1;
      char sync2;

      bool checkACK(tbyte_array& arr, uint8_t cmd_set, uint8_t cmd);

      bool sendNoDataCmd(uint8_t cmd_set, uint8_t cmd);

      std::vector<std::string> error_desc;

      void errMsg(std::string msg);

      class ReadSetupParameters
          {
          public:
              ReadSetupParameters(): fixedSize(false), delim(""), data(0), size(0) {}

              explicit ReadSetupParameters(const std::string& delim):
                      fixedSize(false), delim(delim), data(0), size(0) { }

              ReadSetupParameters(char *data, size_t size): fixedSize(true),
                      delim(""), data(data), size(size) { }

              //Using default copy constructor, operator=

              bool fixedSize; ///< True if need to read a fixed number of parameters
              std::string delim; ///< String end delimiter (valid if fixedSize=false)
              char *data; ///< Pointer to data array (valid if fixedSize=true)
              size_t size; ///< Array size (valid if fixedSize=true)
          };

      void performReadSetup(const ReadSetupParameters& param);

      boost::asio::io_service io;
      boost::asio::serial_port serial;
      boost::asio::deadline_timer timer; // timer for timeout
      boost::posix_time::time_duration timeout;
      boost::asio::streambuf readData;

      size_t bytesTransferred;
      ReadSetupParameters setupParameters;

      void read(char *data, size_t size);

      void write(const tbyte_array& data);

      tbyte_array read(size_t size);

      void timeoutExpired(const boost::system::error_code& error);

      void readCompleted(const boost::system::error_code& error, const size_t bytesTransferred);

      enum ReadResult { resultInProgress, resultSuccess, resultError, resultTimeoutExpired};

      enum ReadResult result;

      float extractFloat(char* addr);


  };

} // namespace

#endif
