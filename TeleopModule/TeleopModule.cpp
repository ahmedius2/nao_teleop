#include "TeleopModule.h"

#include <qi/log.hpp>
#include <alvision/alimage.h>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>

TeleopModule::TeleopModule(boost::shared_ptr<AL::ALBroker> broker,
                           const std::string& name)
    : AL::ALModule(broker, name)
{

    // Describe the module here. This will appear on the webpage
    setModuleDescription("Teleoperation module.");

    // If you had other methods, you could bind them here...
    /**
    * Bound methods can only take const ref arguments of basic types,
    * or AL::ALValue or return basic types or an AL::ALValue.
    */

    cs = new CameraServer("TeleopModule",AL::kQVGA,AL::kRGBColorSpace,30,13333,
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
    qi::log::setSynchronousLog(true);
    //qi::log::setVerbosity(qi::LogLevel_Debug);
    qiLogInfo("Teleoperation module initializing");
}


