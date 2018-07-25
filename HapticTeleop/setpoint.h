#ifndef SETPOINT_H
#define SETPOINT_H

#include "squarewavegenerator.h"
#include "ColumnVector.hpp"


/**
 * Generates Square Wave form.
 */
class SetPoint
{
public:
    SetPoint();

    ColumnVector<5> find_wd(ColumnVector<5> wd,ColumnVector<5> w,
                            double pSamplingPeriod );

    double v_lim_mult;

private:
	ColumnVector<5> linear_interpolator(ColumnVector<5>& final, 
		ColumnVector<5>& current, ColumnVector<5>& speed, double sample_time);

	ColumnVector<5> toSI;
	ColumnVector<5> v_lim;
};

#endif // SQUAREWAVEGENERATOR_H
