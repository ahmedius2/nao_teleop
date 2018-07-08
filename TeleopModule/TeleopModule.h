#ifndef TELEOPMODULE_H
# define TELEOPMODULE_H

#include <alcommon/almodule.h>

#include <alproxies/altexttospeechproxy.h>
#include <alproxies/almotionproxy.h>

#include <boost/thread.hpp>
#include <boost/atomic.hpp>


namespace AL
{
// This is a forward declaration of AL:ALBroker which
// avoids including <alcommon/albroker.h> in this header
class ALBroker;
}

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
        STOPPED,
        WALK,
        HEAD,
        CARTESIAN_HANDS,
        CARTESIAN_HANDS_WB // Whole body hands
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
    // set this on change
    // change teleoperation mode
    void changeMode(const TeleopMode &newMode);
    // position = [x, y, z, roll, pitch, yaw]
    // set this periodially on the operator side
    void setOpCoords(const AL::ALValue &coordinates);
    // keyboard inputs
    void setOpInputsForCurMode(const AL::ALValue &inputs);

private:
    AL::ALMotionProxy motionProxy;
    AL::ALTextToSpeechProxy ttsProxy;

    boost::atomic<TeleopMode> currentMode;
    boost::mutex mtx;
    float lastOpCoords[6];
};
#endif // TELEOPMODULE_H
