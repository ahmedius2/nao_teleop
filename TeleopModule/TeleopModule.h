#ifndef TELEOPMODULE_H
# define TELEOPMODULE_H

#include <alcommon/almodule.h>
#include <alcommon/albroker.h>

#include <alproxies/altexttospeechproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/albasicawarenessproxy.h>


#include <boost/thread.hpp>
#include <boost/atomic.hpp>

#include "cameraserver.h"


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
        LLEG,
        RLEG,
        RARM,
        BOTH_ARMS,
        WALK,
        STOP
    };


    /**
   * Overloading ALModule::init().
   * This is called right after the module has been loaded
   */
    virtual void init();

    // start teleoperation
    void startTeleop();
    // stop teleoperation
    void stopTeleop();
    // change teleoperation mode
    void setMode(const AL::ALValue &newMode);
    void openOrCloseRHand();
    void openOrCloseLHand();
    void switchWholeBody();
    // position = [x, y, z, roll, pitch, yaw]
    // set this periodially on the operator side
    void setOpCoords(const AL::ALValue &coordinates);

private:
    AL::ALMotionProxy motionProxy;
    AL::ALRobotPostureProxy robotPostureProxy;
    AL::ALTextToSpeechProxy ttsProxy;
    AL::ALBasicAwarenessProxy awareness;

    CameraServer *cs;

    TeleopMode currentMode;
    boost::atomic<TeleopMode> nextMode;
    boost::atomic<bool> wholeBodySwitch;
    float lastOpCoords[6];
    boost::mutex mtxOpCoords;
    boost::thread actionThread;

    void actionThreadFunc();

};
#endif // TELEOPMODULE_H
