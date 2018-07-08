#ifndef SQUAREWAVEGENERATOR_H
#define SQUAREWAVEGENERATOR_H

#include <math.h>
#ifndef M_PI
 #define M_PI 3.14159265358979323846
#endif


/**
 * Generates Square Wave form.
 */
class SquareWaveGenerator
{
public:
    SquareWaveGenerator(double pFrequency = 1, double pAmplitude = 1);

    double operator() ( double pTime );

    // Frequency in Hz.
    double frequency() { return mFrequency; }
    void setFrequency( double pFrequency ) { mFrequency = pFrequency; }

    // Amplitude.
    double amplitude() { return mAmplitude; }
    void setAmplitude( double pAmplitude) { mAmplitude = pAmplitude; }

    // Inverted.
    bool isInverted() { return mInverted == -1 ? false : true; }
    void setInverted( bool pInverted ) { mInverted = (pInverted == true ? 1 : -1); }

    // Time Delay in seconds.
    double timeDelay() { return mTimeDelay; }
    void setTimeDelay( double pTimeDelay, double pInitial = 0) { mTimeDelay = pTimeDelay; mTimeDelayInitialValue = pInitial; }

private:

    double mFrequency;
    double mAmplitude;
    int mInverted;

    double mTimeDelay;
    double mTimeDelayInitialValue;

};

#endif // SQUAREWAVEGENERATOR_H
