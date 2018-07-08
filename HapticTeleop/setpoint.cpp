#include "setpoint.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

SetPoint::SetPoint()
{
	squareWave_x.setFrequency( 0.25 );
	
	squareWave_y.setFrequency( 0.25 );
	squareWave_y.setTimeDelay( 1 );

	squareWave_z.setFrequency( 0.125 );
	squareWave_z.setTimeDelay( 0.25 );

	squareWave_yaw.setFrequency( 0.5 );
	squareWave_yaw.setTimeDelay( 0.25 );

	squareWave_roll.setFrequency( 0.5 );
	squareWave_roll.setTimeDelay( 0.5 );

		
	wd_0 = 0, 0, 0, 0, 0;	
	toSI = 1e-3, 1e-3, 1e-3, M_PI/180, M_PI/180;	
	v_lim = 50.0 * toSI;
    current = wd_0;
    trajectory_space = 30, 30, 30, 20, 20;
}

void SetPoint::reset()
{
    current = wd_0;
}

ColumnVector<5> SetPoint::wd( double pTime, double pSamplingPeriod )
{
	ColumnVector<5> wd;
	wd = 
        squareWave_x( pTime ) * trajectory_space.getElement(1),
        squareWave_y( pTime ) * trajectory_space.getElement(2),
        squareWave_z( pTime ) * trajectory_space.getElement(3),
        squareWave_yaw( pTime ) * trajectory_space.getElement(4),
        squareWave_roll( pTime ) * trajectory_space.getElement(5);

	wd = elementProduct( wd, toSI );

	current = linear_interpolator( wd, current, v_lim, pSamplingPeriod );

	return current;
}

ColumnVector<5> SetPoint::linear_interpolator(ColumnVector<5>& final, 
											  ColumnVector<5>& current, 
											  ColumnVector<5>& speed, 
											  double sample_time)
{
	ColumnVector<5> step = speed * sample_time;

	ColumnVector<5> temp;
	temp = 
		sgn( final(1) - current(1) ),
		sgn( final(2) - current(2) ),
		sgn( final(3) - current(3) ),
		sgn( final(4) - current(4) ),
		sgn( final(5) - current(5) );

	ColumnVector<5> inter;
	inter = current + elementProduct( temp, step );

	for ( int i = 1; i <= 5; ++i )
        if( fabs( final(i)-current(i) ) < step(i) )
			inter(i) = final(i);

	return inter;
}
