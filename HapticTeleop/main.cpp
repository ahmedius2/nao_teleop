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


#define NAO_IP_ADDR "10.1.18.18"
#define ALMOTION_FREQUENCY_HZ 50

using namespace Hardware;
using namespace std;

// INSEAD OF DEFINING THIS ENUM, INCLUDE TELEOPMODULE.H
enum TeleopMode{
    STOP, // eaaaah, conflict
    WALK,
    HEAD,
    CARTESIAN_HANDS,
    CARTESIAN_HANDS_WB // Whole body hands
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

    
    // ----- Control Variables -----
    double bias;
    
    // ----- Variables -----
    HapticWand hapticWand;
    SetPoint setPoint;
    PositionController positionController;
    ColumnVector<5> firstSample;
    std::vector<float> opCoords;


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

    registerControlVariable( &bias, "Bias", 1, 1);
    registerControlVariable( setPoint.trajectory_space.getElementsPointer(),
                             "trajectory_space", 1, 5 );

    hapticWand.openDevice();              // Open the q8 card
    hapticWand.calibrateWand();     // Calibrate the haptic wand

    bias = 0.2;

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

    setPoint.reset();

    firstSample =
            hapticWand.firstSample()[0],
            hapticWand.firstSample()[1],
            hapticWand.firstSample()[2],
            hapticWand.firstSample()[3],
            hapticWand.firstSample()[4];

    positionController.reset( firstSample, period() );

    teleopProxy->callVoid("changeMode",AL::ALValue((int)CARTESIAN_HANDS));

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

    static unsigned setOpCoordsFreqCounter = 0;
    if(++setOpCoordsFreqCounter == frequency() / ALMOTION_FREQUENCY_HZ){ 
        // to reach nao, we need to call methods from teleop proxy
        // but these methods involves network communication and may block caller
        // so to do this we made a queue of functions and pushed functions in it
        // and the communication thread popped and called them
        if(commDataMutex.try_lock()){
            // Op X is Nao Y, Op Y is Nao X
            // 0.25 comes from haptic device default position
            opCoords = {(float)(w(2)*bias), (float)(w(1)*bias),
                        (float)(w(3)*bias), (float)(w(4)*bias),
                        (float)(w(5)*bias), 0.0f};

            // push the function call to queue
            commQueue.push([this](){
                teleopProxy->callVoid("setOpCoords", AL::ALValue(opCoords));
            });

            commDataMutex.unlock();
            commCondVar.notify_one();
            setOpCoordsFreqCounter = 0;
        }
        else{
            //couldn't lock, try again next loop
            setOpCoordsFreqCounter -= 1;
        }
    }

    // Desired position.
    wd = /*setPoint.wd( elapsedTime() , period() ) +*/ firstSample;

    // calculate forces.
    ColumnVector<5> F;
    F = positionController.force( w, wd );

    hapticWand.generateForces( period(), jointAngles, F.getElementsPointer() );

    return 0;
}


/**
 * Called when a timed run ends or the STOP button is pushed from zenom.
 *
 * @return Return non-zero to indicate an error.
 */
int HapticTeleop::stop()
{
    teleopProxy->callVoid("stopTeleop");

    hapticWand.disableWand();

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

