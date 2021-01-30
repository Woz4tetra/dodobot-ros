#include "db_object_filter/NonLinearMeasurementPdf.h"

#define MEASMODEL_NUMCONDARGUMENTS_MOBILE 1
#define MEASMODEL_DIMENSION_MOBILE        3

using namespace BFL;
using namespace MatrixWrapper;

NonlinearMeasurementPdf::NonlinearMeasurementPdf(const Gaussian& measNoise)
: ConditionalPdf<ColumnVector,ColumnVector>(MEASMODEL_DIMENSION_MOBILE,MEASMODEL_NUMCONDARGUMENTS_MOBILE)
{
    _measNoise = measNoise;
}


NonlinearMeasurementPdf::~NonlinearMeasurementPdf(){}

Probability NonlinearMeasurementPdf::ProbabilityGet(const ColumnVector& measurement) const
{
    ColumnVector state = ConditionalArgumentGet(0);

    ColumnVector expected_measurement(3);

    // Compute the range according to the map
    expected_measurement(1) = map_calc_range(state(1), state(2), state(3) + 0, 10.0); //front
    expected_measurement(2) = map_calc_range(state(1), state(2), state(3) + M_PI_2, 10.0); //left
    expected_measurement(2) = map_calc_range(state(1), state(2), state(3) - M_PI_2, 10.0); //right

    Probability prb = _measNoise.ProbabilityGet(measurement-expected_measurement);

    return prb;
}
