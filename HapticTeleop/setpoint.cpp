#include "setpoint.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

SetPoint::SetPoint()
{	
	toSI = 1e-3, 1e-3, 1e-3, M_PI/180, M_PI/180;	
    v_lim_mult = 75.0;

}

ColumnVector<5> SetPoint::find_wd(ColumnVector<5> wd,
                             ColumnVector<5> w,
                             double pSamplingPeriod )
{
    v_lim = v_lim_mult * toSI;
    return linear_interpolator( wd, w, v_lim, pSamplingPeriod );
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
        if( fabs( final(i)-current(i) ) < step(i) ){
            inter(i) = final(i);
        }

	return inter;
}
