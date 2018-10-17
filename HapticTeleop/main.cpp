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

#include "HapticTeleop.h"

/**
 * The main function starts the control program
 */
int main( int argc, char *argv[] )
{
    HapticTeleop c;
    c.run( argc, argv );
    
    return 0;
}

