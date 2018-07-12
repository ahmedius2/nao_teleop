#include <iostream>
#include <alerror/alerror.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>

int main(int argc, char* argv[]) {
  if(argc != 2)
  {
    std::cerr << "Wrong number of arguments!" << std::endl;
    std::cerr << "Usage: testTeleopModule NAO_IP" << std::endl;
    exit(2);
  }

  const std::string robotIP = argv[1];
  int port = 9559;

  try {
    /** Create a generic proxy to "TeleopModule" module.
    * Arguments for the constructor are
    * - name of the module
    * - string containing the IP adress of the robot
    * - port (default is 9559)
    */
    boost::shared_ptr<AL::ALBroker> broker =
      AL::ALBroker::createBroker("MyBroker", "", 0, robotIP, port);

    boost::shared_ptr<AL::ALProxy> testProxy
      = boost::shared_ptr<AL::ALProxy>(new AL::ALProxy(broker, "TeleopModule"));


    testProxy->callVoid("startTeleop");

    testProxy->callVoid("stopTeleop");
  }
  catch (const AL::ALError& e) {
    std::cerr << e.what() << std::endl;
  }
}
