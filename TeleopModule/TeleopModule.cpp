#include "TeleopModule.h"

#include <qi/log.hpp>
#include <alvision/alimage.h>
#include <string>
TeleopModule::TeleopModule(boost::shared_ptr<AL::ALBroker> broker,
                           const std::string& name)
    : AL::ALModule(broker, name),
      motionProxy(getParentBroker()),
      ttsProxy(getParentBroker()),
      currentMode(STOPPED)
{
    lastOpCoords[0] = 0; lastOpCoords[1] = 0; lastOpCoords[2] = 0;
    lastOpCoords[3] = 0; lastOpCoords[4] = 0; lastOpCoords[5] = 0;

    // Describe the module here. This will appear on the webpage
    setModuleDescription("Teleoperation module.");

    functionName("startTeleop", getName(), "Starts the teleoperation");
    BIND_METHOD(TeleopModule::startTeleop);

    functionName("stopTeleop", getName(), "Stops the teleoperation");
    BIND_METHOD(TeleopModule::stopTeleop);

    functionName("changeMode", getName(), "Changes teleoperation mode.");
    addParam("newMode", "Selected mode,(walk,hands,wbhands");
    BIND_METHOD(TeleopModule::changeMode);

    functionName("setOpCoords",getName(),"Sets operator world coordinates");
    //setReturn("boolean", "return true");
    addParam("coordinates", "double vector [x y z roll pitch yaw]");
    BIND_METHOD(TeleopModule::setOpCoords);

    functionName("setOpInputsForCurMode",getName(),"Sets current inputs");
    addParam("inputs", "bool vector of keys, true is pushed and false"
                       " is released. Size may change for every mode");
    BIND_METHOD(TeleopModule::setOpInputsForCurMode);

    // If you had other methods, you could bind them here...
    /**
    * Bound methods can only take const ref arguments of basic types,
    * or AL::ALValue or return basic types or an AL::ALValue.
    */

    cs = new CameraServer("TeleopModule",AL::kVGA,AL::kRGBColorSpace,30,12345,
                          getParentBroker());
}

TeleopModule::~TeleopModule()
{
    delete cs;
    qiLogDebug("Teleoperation module destructed");
}

void TeleopModule::init()
{
    /**
   * Init is called just after construction.
   * Do something or not
   */
    ttsProxy.say("Teleoperation module initializing");
    qi::log::setSynchronousLog(true);
    qi::log::setVerbosity(qi::LogLevel_Debug);
    qiLogDebug("Teleoperation module initializing");
}


void TeleopModule::startTeleop()
{
    ttsProxy.say("Starting teleoperation");


}

void TeleopModule::stopTeleop()
{

    ttsProxy.say("Teleoperation stopped");
}

void TeleopModule::changeMode(const TeleopMode &newMode)
{
    currentMode = newMode;
}

void TeleopModule::setOpCoords(const AL::ALValue &coordinates)
{
    boost::lock_guard<boost::mutex> guard(mtx);
    for(int i=0; i<6; ++i)
        lastOpCoords[i] = coordinates[i];
}

void TeleopModule::setOpInputsForCurMode(const AL::ALValue &inputs)
{

}
