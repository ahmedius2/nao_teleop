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

#include <controlbase.h>
#include <hapticwand.h>
#include <hapticwand_utils.h>
#include <alproxies/altexttospeechproxy.h>
#include "setpoint.h"
#include "positioncontroller.h"

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
    // ----- Log Variables -----
    ColumnVector<5> w;      // current world position.
    ColumnVector<5> wd;     // desired world position.

    
    // ----- Control Variables -----
    
    
    // ----- Variables -----
    HapticWand hapticWand;
    SetPoint setPoint;
    PositionController positionController;
    ColumnVector<5> firstSample;
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
    registerLogVariable( w.getElementsPointer(), "w", 1, 5 );
    registerLogVariable( wd.getElementsPointer(), "wd", 1, 5 );

    registerControlVariable( setPoint.trajectory_space.getElementsPointer(), "trajectory_space", 1, 5 );

    hapticWand.openDevice();              // Open the q8 card
    hapticWand.calibrateWand();     // Calibrate the haptic wand

    AL::ALTextToSpeechProxy tts("nao.local" , 9559);
    tts.say("Hello world");
    
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
    hapticWand.enableWand();

    setPoint.reset();

    firstSample =
            hapticWand.firstSample()[0],
            hapticWand.firstSample()[1],
            hapticWand.firstSample()[2],
            hapticWand.firstSample()[3],
            hapticWand.firstSample()[4];

    positionController.reset( firstSample, period() );

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
    hapticWand.forwardKinematics( jointAngles, w.getElementsPointer() );    // current world position.

    // Desired position.
    wd = setPoint.wd( elapsedTime() , period() ) + firstSample;

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

