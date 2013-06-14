#ifndef MS_3DMGX3_45_HH
#define MS_3DMGX3_45_HH

#include <iostream>
#include <boost/asio/serial_port.hpp> 
#include <boost/asio.hpp>
#include <boost/utility.hpp>

/*
 *
 * Serial communication based on following tutorial: http://www.webalice.it/fede.tft/serial_port/serial_port.html
 *
 *
 */

namespace microstrain_3dm_gx3_45 {

	typedef std::vector<char> tbyte_array;


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

		  BASIC_PING = 0x01,
		  BASIC_SET_TO_IDLE = 0x02,
		  BASIC_GET_DEV_INFO = 0x03,
		  BASIC_GET_DEV_DESC_SETS = 0x04,
		  BASIC_DEV_BUILTIN_TEST = 0x05,
		  BASIC_RESUME = 0x06,
		  BASIC_RESET = 0x7E

	  };

	  enum glob_descs {

		  DESC_ACK = 0xF1

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


      void test();

      bool isOpen() const;

      void closePort();

      void setTimeout(const boost::posix_time::time_duration& t);

      bool ping();

      bool selfTest();
    
    private:

    protected:

      void crc(tbyte_array& arr);
      bool crcCheck(tbyte_array& arr);

      char sync1;
      char sync2;

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


  };

} // namespace

#endif
