#ifndef HAPTICTELEOP_H
#define HAPTICTELEOP_H

#include <string>
#include <vector>
#include <iterator>
#include <thread>
#include <chrono>
#include <mutex>
#include <queue>
#include <functional>
#include <condition_variable>
#include <controlbase.h>
#include <hapticwand.h>
#include <hapticwand_utils.h>
#include "hapticlimits.h"
#include <alvalue/alvalue.h>
#include <alerror/alerror.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/albasicawarenessproxy.h>
#include <alproxies/almemoryproxy.h>
#include <boost/swap.hpp>
#include <almath/tools/almath.h>
#include <almath/tools/altransformhelpers.h>
#include "setpoint.h"
#include "positioncontroller.h"
#include "znm-core_global.h"
#include "NaoKinematics/naolimits.h"

#define NAO_IP_ADDR "10.1.18.14"

#define MATLAB_TCP_PORT 30000

#define ALMOTION_FREQUENCY_HZ 50
#define MANIP_MODES 4 // Manipulation modes
#define ALL_MODES (MANIP_MODES+3)

#define HAPTIC_AXES 5
#define HAPTIC_NUM_OF_JOINTS 6
#define XYZRPY 6
#define HAPTIC_BUTTON_PIN 4

#define FRAME_TORSO 1
#define CONTROL_AXES 63

#define ROUND(x) std::roundf((float)((x) * 100.0) ) / 100.0


using namespace Hardware;
using namespace std;

// INSEAD OF DEFINING THIS ENUM, INCLUDE TELEOPMODULE.H
enum TeleopMode{ HEAD, LARM, RARM, BOTH_ARMS, WALK_TO, WALK_TOWARD, STOP };


class HapticTeleop : public ControlBase
{
public:

    // ----- User Functions -----
    // This functions need to be implemented by the user.
    int initialize();
    int start();
    int doloop();
    int stop();
    int terminate();

private:

    // This Thread handles communication with Nao
    class NetworkTask : public TaskXn
    {
    public:
        NetworkTask(HapticTeleop *htPtr)
            : TaskXn("MessageListenerTask") // rt task
            , ht(htPtr){ }

        void finishTask(){
            runCommTask = false;
            ht->commCondVar.notify_one();

        }

    private:
        HapticTeleop *ht;
        std::atomic<bool> runCommTask;
        void run() override{
            runCommTask = true;
            while(runCommTask){
                std::unique_lock<std::mutex> ulock(ht->commDataMutex);
                ht->commCondVar.wait(ulock,[this](){
                    return ht->commQueue.size() > 0 || !runCommTask;
                });

                while(!ht->commQueue.empty()){
                    // call the stored communication function
                    ht->commQueue.front()();
                    ht->commQueue.pop();
                }

            }

        }
    };

    static bool isManipulationMode(TeleopMode m){
        return (m == HEAD || m == LARM || m == RARM || m == BOTH_ARMS);
    }

    void checkKeysAndSetMode();
    void moveRobot();
    void hapticGoToPosBlocking(double threshold);

    // this queue stores network calls for communication
    std::queue<std::function<void()>> commQueue;
    std::mutex commDataMutex;
    std::condition_variable commCondVar;
    NetworkTask *commTask; // communication thread

    // ----- Log Variables -----
    ColumnVector<5> w;      // current world position.
    ColumnVector<5> wd;     // desired world position.
    ColumnVector<5> wd_inter;
    // ----- Control Variables -----

    // catch keyboard strokes to change mode or any ohter purpose
    double headKey; // HEAD mode
    double cartLHandKey,cartRHandKey; // LARM, RARM, BOTH_ARMS mode
    double openCloseLHandKey,openCloseRHandKey; // Open - Close hand
    double walkToKey; // WALK_TO mode
    double walkTowardKey; // WALK_TOWARD mode
    double wholeBodyKey;
    // Is the button state changed to released from pushed?
    // Make sure the button is released before mode switch
    bool isButtonPushed;

    // ----- Variables -----
    double mappingCoefs[ALL_MODES][HAPTIC_AXES] = {
        {1, 1, 1, 1.8392, 0.1788},  // HEAD
        {0.5, 0.5, 0.5, 1.176,  1}, // LARM
        {0.5, 0.5, 0.5, 1.176,  1}, // RARM
        {0.5, 0.5, 0.5, 1.176,  1}, // BOTH_ARMS
        {1.0/HAPTIC_MAX_X, 1.0/((HAPTIC_MAX_Y-HAPTIC_MIN_Y)/2.0), 1,1,1},// WALK_TO
        {1.0/HAPTIC_MAX_X, 1.0/((HAPTIC_MAX_Y-HAPTIC_MIN_Y)/2.0), 1,1,1},// WALK_TOWARD
        {1, 1, 1, 1,      1} // STOP
    };
    double mappingBias[ALL_MODES][HAPTIC_AXES] = {
        {0, 0, 0, 0, -0.0651454}, //HEAD
        {0, 0, 0, 0, 0}, // LARM
        {0, 0, 0, 0, 0}, // RARM
        {0, 0, 0, 0, 0}, // BOTH_ARMS
        {0, 0, 0, 0, 0}, // WALK_TO
        {0, 0, 0, 0, 0}, // WALK_TOWARD
        {0, 0, 0, 0, 0}  // STOP
    };

    const char *chainStr[NAO_CHAINS] = { "Head","LArm","RArm" };
    AL::Math::Transform initTfRArm, initTfLArm;
    std::vector<float> initRobotPosWalk;

    boost::shared_ptr<AL::ALBroker> broker;
    boost::shared_ptr<AL::ALMotionProxy> motionProxy;
    boost::shared_ptr<AL::ALBasicAwarenessProxy> awarenessProxy;
    boost::shared_ptr<AL::ALMemoryProxy> memoryProxy;

    int matlabTCPSocket;

    bool isHapticWandEnabled;
    bool lHandOpen, rHandOpen;
    bool useWholeBody;
    HapticWand hapticWand;
    SetPoint setPoint;
    PositionController positionController;
    TeleopMode curMode; // Current mode
    ColumnVector<5> lastSamples[MANIP_MODES];
};

#endif
