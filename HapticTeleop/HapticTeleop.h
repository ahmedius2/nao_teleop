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
#include <alproxies/alautonomouslifeproxy.h>
#include <alproxies/alautonomousmovesproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/almemoryproxy.h>
#include <boost/swap.hpp>
#include <almath/tools/almath.h>
#include <almath/tools/altransformhelpers.h>
#include "setpoint.h"
#include "positioncontroller.h"
#include "znm-core_global.h"
#include "naolimits.h"

#define NAO_IP_ADDR "10.1.18.12"
//#define NAO_IP_ADDR "192.168.43.204"

#define MATLAB_TCP_PORT 19345

#define NUM_OF_ARM_ANGLES 6

#define INPUT_CHECK_PERIOD_SEC 0.1
#define FB_IMPOSE_TIME 0.3
#define FB_TOTAL_TIME 3.0

#define MANIP_MODES 4 // Manipulation modes
#define ALL_MODES (MANIP_MODES+2)

#define HAPTIC_AXES 5
#define HAPTIC_NUM_OF_JOINTS 6
#define XYZRPY 6
#define HAPTIC_BUTTON_PIN 4

#define FRAME_TORSO 0
#define CONTROL_AXES 63 // all axes

#define ROUND(x) std::roundf((float)((x) * 100.0) ) / 100.0

enum TeleopMode   { HEAD, LARM,  RARM, BOTH_ARMS, WALK_TOWARD, STOP };
enum FeedbackMode { NO_FEEDBACK_START, FEEDBACK_START,
                    NO_FEEDBACK_CNT, FEEDBACK_CNT
                    };

using namespace Hardware;
using namespace std;

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

    static inline bool isArmMode(TeleopMode m){
        return ( m == LARM || m == RARM || m == BOTH_ARMS);
    }

    inline double elapsedFbTime(){
        return elapsedTime() - fbStartTime;
    }

    void checkInputAndSetMode();
    void moveRobot();
    void openOrCloseHand(bool isOpen, std::string hand);
    inline void enableForceIfDisabled(double dividorStep);
    inline void disableForce();
    bool areOfAnyFeetBumpersPressed();
    void hapticGoToPosBlocking(double threshold);
    void connectToMATLAB();
    void disconnectFromMATLAB();

    inline unsigned msToTicks(unsigned milliseconds){
        return frequency() * milliseconds * 0.001;
    }

    // this queue stores network calls for communication
    std::queue<std::function<void()>> commQueue;
    std::mutex commDataMutex;
    std::condition_variable commCondVar;
    NetworkTask *commTask; // communication thread

    // ----- Log Variables -----
    ColumnVector<5> F;
    ColumnVector<5> w;      // current world position.
    ColumnVector<5> w_temp;
    ColumnVector<5> wd;     // desired world position.
    ColumnVector<5> wd_inter;
    // ----- Control Variables -----

    // get input from user via gui and save it to these control variables
    double modeSelectedByUser;
    double rHandStateFromUser, lHandStateFromUser, wbStateFromUser;
    double distBetwArms;

    bool buttonState = false;

    //predefined angles for arms
    const float predefinedArmAngles[NUM_OF_ARM_ANGLES*2] = {
        1.38503, -0.0193124, -1.54517, -1.37153, 0.0280995, 0,
        1.38503, 0.0193124, 1.54517, 1.37153, -0.0280997, 0};
    std::vector<float> preArmAnglesVec;

    // ----- Variables -----
    double mappingCoefs[ALL_MODES][HAPTIC_AXES] = {
        {1, 1, 1, 1.8392, 0.1788},  // HEAD
        {0.5, 0.5, 0.5, 1.176,  1}, // LARM
        {0.5, 0.5, 0.5, 1.176,  1}, // RARM
        {0.5, 0.5, 0.5, 1.176,  1}, // BOTH_ARMS
        {1.0/HAPTIC_MAX_X, 1.0/((HAPTIC_MAX_Y-HAPTIC_MIN_Y)/2.0), 1,1,1},// WALK_TOWARD
        {1, 1, 1, 1,      1} // STOP
    };
    double mappingBias[ALL_MODES][HAPTIC_AXES] = {
        {0, 0, 0, 0, -0.0651454}, //HEAD
        {0, 0, 0, 0, 0}, // LARM
        {0, 0, 0, 0, 0}, // RARM
        {0, 0, 0, 0, 0}, // BOTH_ARMS
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
    boost::shared_ptr<AL::ALAutonomousMovesProxy> automoProxy;
    //boost::shared_ptr<AL::ALRobotPostureProxy> postureProxy;

    int matlabTCPSocket;

    bool lHandState= false, rHandState = false;
    bool wbState = false;
    HapticWand hapticWand;
    bool hapticWandForceEnabled = false;
    SetPoint setPoint;
    std::atomic<FeedbackMode> curFbMode, newFbMode;
    double manipulability, fbStartTime;
    bool fbCooldown = false;
    const float manipThreshold = 30.0f;
    PositionController positionController;
    double forceDivider = 1.0, forceDividerStep;
    TeleopMode curTeleopMode; // Current mode
    ColumnVector<5> lastSamples[MANIP_MODES];

    std::chrono::system_clock::time_point tm;
};

#endif
