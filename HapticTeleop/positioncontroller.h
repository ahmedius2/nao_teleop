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
    void resetStiffnessAndDamping();
	ColumnVector<5> force( ColumnVector<5>& w, ColumnVector<5>& wd ); 

    ColumnVector<5> stiffness;
    ColumnVector<5> damping;

private:	

	Euler2DigitalFilter< ColumnVector<5> > digitalFilter;
};

#endif // POSITION_CONTROLLER_H
