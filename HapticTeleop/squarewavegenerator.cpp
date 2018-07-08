#include "squarewavegenerator.h"

SquareWaveGenerator::SquareWaveGenerator(double pFrequency, double pAmplitude)
    : mFrequency(pFrequency), mAmplitude(pAmplitude)
{
    setInverted( false );
    setTimeDelay( 0, 0 );
}

double SquareWaveGenerator::operator() ( double pTime )
{
    double v;
    if ( mTimeDelay > pTime )
        v = mTimeDelayInitialValue;
    else
        v = (sin( 2 * M_PI * (pTime - mTimeDelay) * mFrequency ) < 0 ? -1 : 1) * mAmplitude * mInverted;

    return v;
}
