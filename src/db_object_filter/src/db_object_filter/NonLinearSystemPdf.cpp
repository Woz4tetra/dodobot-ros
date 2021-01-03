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


NonlinearSystemPdf::~NonlinearSystemPdf(){}


bool NonlinearSystemPdf::SampleFrom (Sample<ColumnVector>& one_sample, int method, void * args) const
{
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector vel   = ConditionalArgumentGet(1);

    // system update
    state(1) += cos(state(3)) * vel(1);
    state(2) += sin(state(3)) * vel(1);
    state(3) += vel(2);

    // sample from additive noise
    Sample<ColumnVector> noise;
    _additiveNoise.SampleFrom(noise, method, args);

    // store results in one_sample
    one_sample.ValueSet(state + noise.ValueGet());

    return true;
}
