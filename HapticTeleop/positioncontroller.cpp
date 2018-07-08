#include "positioncontroller.h"



PositionController::PositionController()
{
    stiffness = 500, 500, 500, 1, 1;
    damping = 5, 5, 5, 0.1, 0.1;
}

void PositionController::reset( ColumnVector<5>& pFirstSample, double pPeriod )
{
	// Filter Cutoff (rad/s)
    double wn_f = 150;

    // Filter Damping Ratio
    double zeta_f = 1;

    digitalFilter.setSamplingPeriod( pPeriod );
    digitalFilter.setCutOffFrequencyRad( wn_f );
    digitalFilter.setDampingRatio( zeta_f );
    digitalFilter.reset( pFirstSample );
}

ColumnVector<5> PositionController::force( ColumnVector<5>& w, ColumnVector<5>& wd )
{
    ColumnVector<5> F_stiff = wd - w;
	F_stiff = elementProduct(F_stiff, stiffness);

	ColumnVector<5> F_damp = digitalFilter.integrate( w );
	F_damp *= -1;
	F_damp = elementProduct( F_damp, damping );

	return F_stiff + F_damp;
}
