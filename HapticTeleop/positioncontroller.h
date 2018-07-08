#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <ColumnVector.hpp>
#include <Euler2DigitalFilter.hpp>

/**
 * 
 */
class PositionController
{
public:
    PositionController();

	void reset( ColumnVector<5>& pFirstSample, double pPeriod );

	ColumnVector<5> force( ColumnVector<5>& w, ColumnVector<5>& wd ); 

private:	
    ColumnVector<5> stiffness;
    ColumnVector<5> damping;
	Euler2DigitalFilter< ColumnVector<5> > digitalFilter;
};

#endif // POSITION_CONTROLLER_H
