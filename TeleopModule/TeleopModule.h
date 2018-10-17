#ifndef TELEOPMODULE_H
# define TELEOPMODULE_H

#include <alcommon/almodule.h>
#include <alcommon/albroker.h>

#include <alproxies/altexttospeechproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/albasicawarenessproxy.h>
#include <alproxies/almemoryproxy.h>

#include <almath/tools/almath.h>
#include <almath/tools/altransformhelpers.h>

#include <boost/thread.hpp>
#include <boost/atomic.hpp>

#include "cameraserver.h"

#define FRAME_TORSO 1
#define CONTROL_AXES 63

/**
 * This class inherits AL::ALModule. This allows it to bind methods
 * and be run as a remote executable within NAOqi
 */
class TeleopModule : public AL::ALModule
{
public:
    TeleopModule(boost::shared_ptr<AL::ALBroker> broker,
                 const std::string &name);

    virtual ~TeleopModule();

    enum TeleopMode{
        HEAD,
        LARM,
        RARM,
        BOTH_ARMS,
        WALK_TO,
        WALK_TOWARD,
        STOP
    };


    /**
   * Overloading ALModule::init().
   * This is called right after the module has been loaded
   */
    virtual void init();


private:
    CameraServer *cs;

};
#endif // TELEOPMODULE_H
