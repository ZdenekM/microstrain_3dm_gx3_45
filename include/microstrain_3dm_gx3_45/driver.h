#ifndef MS_3DMGX3_45_HH
#define MS_3DMGX3_45_HH

#include <iostream>
#include <boost/asio/serial_port.hpp> 
#include <boost/asio.hpp> 

namespace microstrain_3dm_gx3_45 {

  class IMU
    {
    
    public:
    
      IMU();

      bool openPort(std::string port, unsigned int baud_rate);

      ~IMU();


      void test();
    
    
    private:
    
      boost::asio::io_service io;
      boost::asio::serial_port *serial;
    
    protected:


  };

} // namespace

#endif
