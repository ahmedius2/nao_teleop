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
#include "hapticlimits.h"
#include <alvalue/alvalue.h>
#include <alerror/alerror.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/almotionproxy.h>
#include <boost/swap.hpp>
#include <Eigen/Eigenvalues>
#include "setpoint.h"
#include "positioncontroller.h"
#include "znm-core_global.h"
#include "NaoKinematics/NaoRArm_jacob0.h"
#include "NaoKinematics/naolimits.h"

#define NAO_IP_ADDR "169.254.67.213"
#define ALMOTION_FREQUENCY_HZ 50
#define MANIP_MODES 8 // Manipulation modes
#define HAPTIC_AXES 5
#define HAPTIC_NUM_OF_JOINTS 6
#define XYZRPY 6


using namespace Hardware;
using namespace std;

// INSEAD OF DEFINING THIS ENUM, INCLUDE TELEOPMODULE.H
enum TeleopMode{ HEAD, LARM, LLEG, RLEG, RARM, BOTH_ARMS, STOP, WALK };


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

    // this queue stores network calls for communication
    std::queue<std::function<void()>> commQueue;
    std::mutex commDataMutex;
    std::condition_variable commCondVar;
    NetworkTask *commTask; // communication thread

    // ----- Log Variables -----
    ColumnVector<5> w;      // current world position.
    ColumnVector<5> wd;     // desired world position.
    ColumnVector<5> wd_inter;
    double manipEigvals[XYZRPY];
    
    // ----- Control Variables -----

    // catch keyboard strokes to change mode or any ohter purpose
    double headKey; // HEAD mode
    double cartLHandKey,cartRHandKey; // LARM, RARM, BOTH_ARMS mode
    double openCloseLHandKey,openCloseRHandKey; // Open - Close hand
    double walkKey; // WALK mode
    double stopKey; // STOP mode
    double cartRLegKey, cartLLegKey; //RLEG LREG mode
    double wholeBodyKey;
    
    // ----- Variables -----
    double mappingCoefs[MANIP_MODES][HAPTIC_AXES] = {
        {1, 1, 1, 1.8392, 0.1788},  // HEAD
        {0.5, 0.5, 0.5, 1.176,  1}, // LARM
        {1, 1, 1, 1,      1}, // LLEG
        {1, 1, 1, 1,      1}, // RLEG
        {0.5, 0.5, 0.5, 1.176,  1}, // RARM
        {0.5, 0.5, 0.5, 1.176,  1}, // BOTH_ARMS
        {1, 1, 1, 1,      1}, // STOP
        {1, 1, 1, 1,      1}  // WALK
    };
    double mappingBias[MANIP_MODES][HAPTIC_AXES] = {
        {0, 0, 0, 0, -0.0651454}, //HEAD
        {0, 0, 0, 0, 0}, // LARM
        {0, 0, 0, 0, 0}, // LLEG
        {0, 0, 0, 0, 0}, // RLEG
        {0, 0, 0, 0, 0}, // RARM
        {0, 0, 0, 0, 0}, // BOTH_ARMS
        {0, 0, 0, 0, 0}, // STOP
        {0, 0, 0, 0, 0}  // WALK
    };

    const char *naoChainNames[NAO_NUM_OF_CHAINS] =
                            { "Head", "LArm", "LLeg", "RLeg","RArm" };
    std::vector<float> naoAngles[NAO_NUM_OF_CHAINS];

    boost::shared_ptr<AL::ALBroker> broker;
    boost::shared_ptr<AL::ALProxy>  teleopProxy;
    boost::shared_ptr<AL::ALMotionProxy> motionProxy;

    bool isHapticWandEnabled;
    HapticWand hapticWand;
    SetPoint setPoint;
    PositionController positionController;
    TeleopMode currentMode;
    ColumnVector<5> lastSamples[MANIP_MODES];
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

    motionProxy = boost::shared_ptr<AL::ALMotionProxy>(
                new AL::ALMotionProxy(broker)
                );

    if(motionProxy.get() == 0){
        logFile << "motionProxy is nullptr!" << std::endl;
        logFile.flush();
    }

    naoAngles[HEAD] = std::vector<float>(NAO_NUM_OF_HEAD_JOINTS, 0);
    naoAngles[LARM] = std::vector<float>(NAO_NUM_OF_ARM_JOINTS,  0);
    naoAngles[LLEG] = std::vector<float>(NAO_NUM_OF_LEG_JOINTS,  0);
    naoAngles[RLEG] = std::vector<float>(NAO_NUM_OF_LEG_JOINTS,  0);
    naoAngles[RARM] = std::vector<float>(NAO_NUM_OF_ARM_JOINTS,  0);

    registerLogVariable( w.getElementsPointer(), "w", 1, HAPTIC_AXES );
    registerLogVariable( wd.getElementsPointer(), "wd", 1, HAPTIC_AXES );
    registerLogVariable( wd_inter.getElementsPointer(), "wd_inter",
                         1, HAPTIC_AXES );
    registerLogVariable( manipEigvals, "manipEigvals",1, XYZRPY );

    registerControlVariable( mappingCoefs[LARM], "mappingCoefs[LARM]",1,HAPTIC_AXES);
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
                             "stiffness", 1, HAPTIC_AXES);
    registerControlVariable( positionController.damping.getElementsPointer(),
                             "damping", 1, HAPTIC_AXES);

    openCloseLHandKey = headKey = openCloseRHandKey = cartLHandKey = walkKey =0;
    cartRHandKey = cartLLegKey = stopKey = cartRLegKey = wholeBodyKey = 0;

    hapticWand.openDevice();        // Open the q8 card
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

    hapticWand.enableWand();
    isHapticWandEnabled = true;

    wd = 0,0,0,0,0;
    wd.setElement(2, HAPTIC_START_YPOS);
    wd_inter = wd;
    for(int i=0; i<MANIP_MODES; ++i)
        lastSamples[i]= wd;

    ColumnVector<5> firstSample;
    firstSample =
            hapticWand.firstSample()[0],
            hapticWand.firstSample()[1],
            hapticWand.firstSample()[2],
            hapticWand.firstSample()[3],
            hapticWand.firstSample()[4];

    positionController.reset( firstSample, period() );

    teleopProxy->callVoid("startTeleop");
    teleopProxy->callVoid("setMode",AL::ALValue(currentMode = STOP));

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
    double jointAngles[HAPTIC_NUM_OF_JOINTS]; // joint angles in radians
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
    if(++robotMotionFreqCounter == (frequency() / ALMOTION_FREQUENCY_HZ)){
        // 50 HZ, 20ms period
        // to reach nao, we need to call methods from teleop proxy
        // but these methods involves network communication and may block caller
        // so to do this we made a queue of functions and pushed functions in it
        // and the communication thread popped and called them
        if(commDataMutex.try_lock()){
            static unsigned buttonFreqCounter = 0;
            if(++buttonFreqCounter == 2){ // 40 ms period
                // don't let it to switch whole body while in another mode
                if(!currentMode && wholeBodyKey){
                    commQueue.push([this](){
                        teleopProxy->callVoid("switchWholeBody");
                    });
                    std::cout << "switchWholeBody" << std::endl;
                }

                if(openCloseRHandKey){
                    commQueue.push([this](){
                        teleopProxy->callVoid("openOrCloseRHand");
                    });
                    std::cout << "openCloseRHand" << std::endl;
                }

                if(openCloseLHandKey){
                    commQueue.push([this](){
                        teleopProxy->callVoid("openOrCloseLHand");
                    });
                    std::cout << "openCloseLHand" << std::endl;
                }
                buttonFreqCounter = 0;
            }

            // mode change
            TeleopMode newMode = STOP;
            if(headKey) newMode = HEAD;
            else if(walkKey) newMode = WALK;
            else if(cartRLegKey) newMode = RLEG;
            else if(cartLLegKey) newMode = LLEG;
            else if(cartLHandKey && cartRHandKey) newMode = BOTH_ARMS;
            else if(cartLHandKey) newMode = LARM;
            else if(cartRHandKey) newMode = RARM;
            // save the current w, when we switch to same mode back,
            // we will use it

            // mode switching can only be done while button is released
            if(!isButtonPushed && currentMode != newMode){
                lastSamples[currentMode] = w;
                wd_inter = w;
                currentMode = newMode;
                commQueue.push([this](){
                    // make the mode switch delayed so the haptic wand can have
                    // some time before returning its last position of
                    // selected mode
                    // don't worry, queue will be locked by commThread while
                    // this thread is sleeping so doloop cannot overflow it
                    // with other calls. But the keys won't work while on mode switch
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    teleopProxy->callVoid("setMode",AL::ALValue(currentMode));
                });
                wd = lastSamples[newMode];
            }


            static unsigned posCtrlFreqCounter = 0;
            if(++posCtrlFreqCounter == 50){ // 500 ms period
                // Op X is Nao Y, Op Y is Nao X
                // 0.125 is due to haptic placement, it should be different for
                // walking, like 0.2 static because we need it after the scope ends
                // as commThread will send it
                static std::vector<float> opCoords(6);
                opCoords[0] = (float)((HAPTIC_MAX_Y-w(2))*
                    mappingCoefs[currentMode][1] + mappingBias[currentMode][1]);
                opCoords[1] = (float)(w(1)*mappingCoefs[currentMode][0]+
                    mappingBias[currentMode][0]);
                opCoords[2] = (float)(w(3)*mappingCoefs[currentMode][2]+
                    mappingBias[currentMode][2]);
                opCoords[3] = (float)(w(5)*mappingCoefs[currentMode][4]+
                    mappingBias[currentMode][4]);
                opCoords[4] = (float)(w(4)*mappingCoefs[currentMode][3]+
                    mappingBias[currentMode][3]),
                opCoords[5] = 0.0f;

                // push the function call to queue
                commQueue.push([this](){
                    teleopProxy->callVoid("setOpCoords", AL::ALValue(opCoords));

                    if(currentMode == BOTH_ARMS){
                        naoAngles[LARM] =
                            motionProxy->getAngles(naoChainNames[LARM],false);
                        naoAngles[RARM] =
                            motionProxy->getAngles(naoChainNames[RARM],false);
                    }
                    else if (currentMode != STOP && currentMode != WALK)
                        naoAngles[currentMode] = motionProxy->getAngles(
                                    naoChainNames[currentMode],false);

                });
                posCtrlFreqCounter = 0;
            }

            commDataMutex.unlock();
            commCondVar.notify_one();

            if(currentMode == RARM){
                static Eigen::Matrix<double,XYZRPY,NAO_NUM_OF_ARM_JOINTS>J_RArm;
                static Eigen::Matrix<double,XYZRPY,XYZRPY>JTimesJTranspose;
                NaoRArm_jacob0(J_RArm, naoAngles[RARM]);
                JTimesJTranspose = J_RArm * J_RArm.transpose();
                Eigen::VectorXcd eivals = JTimesJTranspose.eigenvalues();
                for(int i=0; i<XYZRPY; ++i)
                    manipEigvals[i] = real(eivals(i));
            }

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

