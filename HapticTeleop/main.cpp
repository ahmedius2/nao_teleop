/**
 * Zenom - Hard Real-Time Simulation Enviroment
 * @author
 *
 * HapticTeleop
 * The HapticTeleop Example is a world-based position control. The setpoint
 * or desired wand position, is given in Cartesian coordinates and the
 * controller calculates how much current is needed in each motor for
 * the wand to attain this position.
 *
 */

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
#include <alvalue/alvalue.h>
#include <alerror/alerror.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alproxies/altexttospeechproxy.h>
#include <boost/swap.hpp>
#include "setpoint.h"
#include "positioncontroller.h"
#include "znm-core_global.h"


#define NAO_IP_ADDR "10.1.18.16"
#define ALMOTION_FREQUENCY_HZ 50

using namespace Hardware;
using namespace std;

// INSEAD OF DEFINING THIS ENUM, INCLUDE TELEOPMODULE.H
// multiple modes can be active at the same time
enum TeleopMode{
    WALK = 1<<0,
    HEAD = 1<<1,
    CARTESIAN_LHAND = 1<<2,
    CARTESIAN_RHAND = 1<<3,
    CARTESIAN_LLEG = 1<<4,
    CARTESIAN_RLEG = 1<<5
};

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

    // ----- Log Variables -----
    ColumnVector<5> w;      // current world position.
    ColumnVector<5> wd;     // desired world position.
    ColumnVector<5> wd_inter;

    
    // ----- Control Variables -----
    double pos_bias,rot_bias;
    // catch keyboard strokes to change mode or any ohter purpose
    double headKey; // HEAD mode
    double cartLHandKey,cartRHandKey; // CARTESIAN_HANDS mode
    double openCloseLHandKey,openCloseRHandKey; // Open - Close hand
    double walkKey; // WALK mode
    double stopKey; // STOP mode
    double cartRLegKey, cartLLegKey;
    double wholeBodyKey;
    
    // ----- Variables -----
    HapticWand hapticWand;
    bool isHapticWandEnabled;
    SetPoint setPoint;
    PositionController positionController;
    int currentModes;
    ColumnVector<5> firstSample;
    std::vector<float> opCoords;
    ColumnVector<5> lastSampleHead, lastSampleRHand,lastSampleLHand,
            lastSampleBothHands,lastSampleRLeg,lastSampleLLeg;

    // this queue stores network calls for communication
    std::queue<std::function<void()>> commQueue;
    std::mutex commDataMutex;
    std::condition_variable commCondVar;
    NetworkTask *commTask; // communication thread

    boost::shared_ptr<AL::ALBroker> broker;
    boost::shared_ptr<AL::ALProxy>  teleopProxy;
};

/**
 * This function is called when the control program is loaded to zenom.
 * Use this function to register control parameters, to register log variables
 * and to initialize control parameters.
 *
 * @return Return non-zero to indicate an error.
 */
int HapticTeleop::initialize()
{
    broker = AL::ALBroker::createBroker("MyBroker", "", 0, NAO_IP_ADDR, 9559);

    teleopProxy = boost::shared_ptr<AL::ALProxy>(
                    new AL::ALProxy(broker, "TeleopModule")
                  );

    registerLogVariable( w.getElementsPointer(), "w", 1, 5 );
    registerLogVariable( wd.getElementsPointer(), "wd", 1, 5 );
    registerLogVariable( wd_inter.getElementsPointer(), "wd_inter", 1, 5 );

    registerControlVariable( &pos_bias, "Position Bias", 1, 1);
    registerControlVariable( &rot_bias, "Rotation Bias", 1, 1);
    registerControlVariable( &setPoint.v_lim_mult, "v_lim_mult",1,1);
    registerControlVariable( &openCloseLHandKey, "key_q", 1, 1);
    registerControlVariable( &headKey, "key_w", 1, 1);
    registerControlVariable( &openCloseRHandKey, "key_e", 1, 1);
    registerControlVariable( &cartLHandKey, "key_a", 1, 1);
    registerControlVariable( &walkKey, "key_s", 1, 1);
    registerControlVariable( &cartRHandKey, "key_d", 1, 1);
    registerControlVariable( &cartLLegKey, "key_z", 1, 1);
    registerControlVariable( &stopKey, "key_x", 1, 1);
    registerControlVariable( &cartRLegKey, "key_c", 1, 1);
    registerControlVariable( &wholeBodyKey, "key_f", 1, 1);
    registerControlVariable( positionController.stiffness.getElementsPointer(),
                             "stiffness", 1, 5);
    registerControlVariable( positionController.damping.getElementsPointer(),
                             "damping", 1, 5);

    pos_bias = 0.5;
    rot_bias = 0.5;
    openCloseLHandKey = headKey = openCloseRHandKey = cartLHandKey = walkKey =0;
    cartRHandKey = cartLLegKey = stopKey = cartRLegKey = wholeBodyKey = 0;

    hapticWand.openDevice();              // Open the q8 card
    hapticWand.calibrateWand();     // Calibrate the haptic wand

    commTask = new NetworkTask(this);
    commTask->runTask();
    return 0;
}

/**
 * This function is called when the START button is pushed from zenom.
 *
 * @return If you return 0, the control starts and the doloop() function is
 * called periodically. If you return nonzero, the control will not start.
 */
int HapticTeleop::start()
{

    if(frequency() < ALMOTION_FREQUENCY_HZ){
        std::cerr << "Your frequency must be at least 50Hz !" << std::endl;
        return -1;
    }

    teleopProxy->callVoid("startTeleop");

    hapticWand.enableWand();
    isHapticWandEnabled = true;

    firstSample =
            hapticWand.firstSample()[0],
            hapticWand.firstSample()[1],
            hapticWand.firstSample()[2],
            hapticWand.firstSample()[3],
            hapticWand.firstSample()[4];

    lastSampleBothHands = 0,0.2,0,0,0;
    lastSampleLHand = 0,0.2,0,0,0;
    lastSampleRHand = 0,0.2,0,0,0;
    lastSampleRLeg = 0,0.2,0,0,0;
    lastSampleLLeg = 0,0.2,0,0,0;
    lastSampleHead = 0,0.2,0,0,0;
    wd_inter =  0,0.2,0,0,0;
    wd = 0,0.2,0,0,0;

    positionController.reset( firstSample, period() );

    teleopProxy->callVoid("setModes",AL::ALValue(currentModes = 0));

    return 0;
}


/**
 * This function is called periodically (as specified by the control frequency).
 * The useful functions that you can call used in doloop() are listed below.
 *
 * frequency()          returns frequency of simulation.
 * period()             returns period of simulation.
 * duration()           returns duration of simulation.
 * simTicks()           returns elapsed simulation ticks.
 * simTimeInNano()      returns elapsed simulation time in nano seconds.
 * simTimeInMiliSec()   returns elapsed simulation time in miliseconds.
 * simTimeInSec()       returns elapsed simulation time in seconds.
 * overruns()           returns the count of overruns.
 *
 * @return If you return 0, the control will continue to execute. If you return
 * nonzero, the control will abort and stop() function will be called.
 */
int HapticTeleop::doloop()
{
    double jointAngles[6];        // joint angles in radians
    hapticWand.jointAngles( jointAngles );
    // current world position.
    hapticWand.forwardKinematics( jointAngles, w.getElementsPointer() );

    // Is the button state changed to released from pushed?
    // Make sure the button is released before mode switch
    static bool isButtonPushed = false;
    if(isButtonPushed && hapticWand.readDigital(4)){
        wd = wd_inter = w;
    }

    isButtonPushed = !hapticWand.readDigital(4);

    static unsigned robotMotionFreqCounter = 0;
    if(++robotMotionFreqCounter == frequency() / ALMOTION_FREQUENCY_HZ){

        // to reach nao, we need to call methods from teleop proxy
        // but these methods involves network communication and may block caller
        // so to do this we made a queue of functions and pushed functions in it
        // and the communication thread popped and called them
        if(commDataMutex.try_lock()){
            // don't let it to switch whole body while in another mode
            if(!currentModes && wholeBodyKey){
                commQueue.push([this](){
                    teleopProxy->callVoid("switchWholeBody");
                });
                std::cout << "switchWholeBody" << std::endl;
            }

            if(openCloseRHandKey)
                commQueue.push([this](){
                    teleopProxy->callVoid("openOrCloseRHand");
                });

            if(openCloseLHandKey)
                commQueue.push([this](){
                    teleopProxy->callVoid("openOrCloseLHand");
                });

            // mode change
            int newModes=0;
            newModes |= walkKey ? WALK : 0;
            newModes |= (!newModes && headKey) ? HEAD : 0;
            newModes |= (!newModes && cartRHandKey) ? CARTESIAN_RHAND : 0;
            newModes |= (!newModes && cartLHandKey) ? CARTESIAN_LHAND : 0;
            newModes |= (!newModes && cartRLegKey)  ? CARTESIAN_RLEG  : 0;
            newModes |= (!newModes && cartLLegKey)  ? CARTESIAN_LLEG  : 0;
            // The combinations can be:
            // Only head, Only left hand, only right hand, both hands,
            // only left leg, and only right leg
            // save the current w, when we switch to same mode back,
            // we will use it

            // mode switching can only be done while button is released
            if(!isButtonPushed && currentModes != newModes){
                switch(currentModes){// clear head bit
                case HEAD:
                    lastSampleHead = w;
                    break;
                case CARTESIAN_RHAND | CARTESIAN_LHAND:
                    lastSampleBothHands = w;
                    break;
                case CARTESIAN_RHAND:
                    lastSampleRHand = w;
                    break;
                case CARTESIAN_LHAND:
                    lastSampleLHand = w;
                    break;
                case CARTESIAN_RLEG:
                    lastSampleRLeg = w;
                    break;
                case CARTESIAN_LLEG:
                    lastSampleLLeg = w;
                    break;
                }
                wd_inter = w;
                currentModes = newModes;
                commQueue.push([this](){
                    // make the mode switch delayed so the haptic wand can have
                    // some time before returning its last position of
                    // selected mode
                    // don't worry, queue will be locked by commThread while
                    // this thread is sleeping so doloop cannot overflow it
                    // with setOpCoords calls
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    teleopProxy->callVoid("setModes",AL::ALValue(currentModes));
                });
                switch(newModes){
                case HEAD:
                    wd = lastSampleHead;
                    break;
                case WALK:
                    wd = 0,0.2,0,0,0;
                case CARTESIAN_RHAND | CARTESIAN_LHAND:
                    wd = lastSampleBothHands;
                    break;
                case CARTESIAN_RHAND:
                    wd = lastSampleRHand;
                    break;
                case CARTESIAN_LHAND:
                    wd = lastSampleLHand;
                    break;
                case CARTESIAN_RLEG:
                    wd = lastSampleRLeg;
                    break;
                case CARTESIAN_LLEG:
                    wd = lastSampleLLeg;
                    break;
                default:
                    break;
                }

            }
            // Op X is Nao Y, Op Y is Nao X
            // 0.25 comes from haptic device default position
            opCoords = {(float)(w(2)*pos_bias), (float)(w(1)*pos_bias),
                        (float)(w(3)*pos_bias), (float)(w(4)*rot_bias),
                        (float)(w(5)*rot_bias), 0.0f};
            // The bias can be different for 4 and 5, as they are for head

            // push the function call to queue
            commQueue.push([this](){
                teleopProxy->callVoid("setOpCoords", AL::ALValue(opCoords));
            });

            commDataMutex.unlock();
            commCondVar.notify_one();
            robotMotionFreqCounter = 0;
        }
        else{
            //couldn't lock, try again next loop
            robotMotionFreqCounter -= 1;
        }
    }


    if(!isButtonPushed){
        if(!isHapticWandEnabled){
            hapticWand.enableWand();
            isHapticWandEnabled = true;
        }

        // Desired position.
        wd_inter = setPoint.find_wd(wd, wd_inter, period() );
        ColumnVector<5> F = positionController.force( w, wd_inter );
        // I need to run RealWriteDigital example for once before start
        // but why!
        hapticWand.generateForces( period(), jointAngles, F.getElementsPointer() );
    }
    else if(isHapticWandEnabled){
        hapticWand.disableWand();
        isHapticWandEnabled = false;
    }

    return 0;
}


/**
 * Called when a timed run ends or the STOP button is pushed from zenom.
 *
 * @return Return non-zero to indicate an error.
 */
int HapticTeleop::stop()
{

    hapticWand.disableWand();

    teleopProxy->callVoid("stopTeleop");

    return 0;
}


/**
 * This function is called when the control is unloaded. It happens when
 * the user loads a new control program or exits.
 *
 * @return Return non-zero to indicate an error.
 */
int HapticTeleop::terminate()
{

    hapticWand.closeDevice();

    commTask->finishTask();
    delete commTask;
    
    return 0;
}


/**
 * The main function starts the control program
 */
int main( int argc, char *argv[] )
{
    HapticTeleop c;
    c.run( argc, argv );
    
    return 0;
}

