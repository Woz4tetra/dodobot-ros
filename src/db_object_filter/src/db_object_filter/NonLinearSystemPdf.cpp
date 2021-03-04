#include "db_object_filter/NonLinearSystemPdf.h"

#define SYSMODEL_NUMCONDARGUMENTS_MOBILE 2
#define SYSMODEL_DIMENSION_MOBILE        3

using namespace BFL;
using namespace MatrixWrapper;

NonlinearSystemPdf::NonlinearSystemPdf(const Gaussian& additiveNoise)
: ConditionalPdf<ColumnVector,ColumnVector>(SYSMODEL_DIMENSION_MOBILE,SYSMODEL_NUMCONDARGUMENTS_MOBILE)
{
    _additiveNoise = additiveNoise;
}


NonlinearSystemPdf::~NonlinearSystemPdf() {  }


bool NonlinearSystemPdf::SampleFrom(Sample<ColumnVector> &one_sample, int method, void *args) const
{
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector vel   = ConditionalArgumentGet(1);

    // vel is the input vector:
    // [vx * dt, vt * dt]

    ROS_INFO("vel(1): %f, vel(2): %f", vel(1), vel(2));
    ROS_INFO("state(1): %f, state(2): %f, state(3): %f", state(1), state(2), state(3));

    // system update
    state(1) += state(1) * cos(vel(2)) - state(2) * sin(vel(2));
    state(2) += state(1) * sin(vel(2)) + state(2) * cos(vel(2));
    state(3) = 0.0;  // Z component of input is always 0

    // sample from additive noise
    _additiveNoise.SampleFrom(_noise, method, args);

    // store results in one_sample
    one_sample.ValueSet(state + _noise.ValueGet());

    return true;
}
