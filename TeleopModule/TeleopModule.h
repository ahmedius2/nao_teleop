#ifndef TELEOPMODULE_H
# define TELEOPMODULE_H

#include <alcommon/almodule.h>
#include <alcommon/albroker.h>

#include <alproxies/altexttospeechproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>


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

    // multiple modes can be active at the same time
    enum TeleopMode{
        WALK = 1<<0,
        HEAD = 1<<1,
        CARTESIAN_LHAND = 1<<2,
        CARTESIAN_RHAND = 1<<3,
        CARTESIAN_LLEG = 1<<4,
        CARTESIAN_RLEG = 1<<5
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
    void setModes(const AL::ALValue &newModes);
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

    CameraServer *cs;

    int currentModes;
    boost::atomic<int> nextModes;
    float lastOpCoords[6];
    boost::mutex mtxOpCoords;
    boost::thread actionThread;

    void actionThreadFunc();

};
#endif // TELEOPMODULE_H
