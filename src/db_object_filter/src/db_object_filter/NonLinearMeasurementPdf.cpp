#include "db_object_filter/NonLinearMeasurementPdf.h"

#define MEASMODEL_NUMCONDARGUMENTS_MOBILE 1
#define MEASMODEL_DIMENSION_MOBILE        3

using namespace BFL;
using namespace MatrixWrapper;

NonlinearMeasurementPdf::NonlinearMeasurementPdf(const Gaussian& measNoise)
: ConditionalPdf<ColumnVector,ColumnVector>(MEASMODEL_DIMENSION_MOBILE, MEASMODEL_NUMCONDARGUMENTS_MOBILE)
{
    _measNoise = measNoise;
}


NonlinearMeasurementPdf::~NonlinearMeasurementPdf() {  }

Probability NonlinearMeasurementPdf::ProbabilityGet(const ColumnVector& measurement) const
{
    ColumnVector state = ConditionalArgumentGet(0);

    ColumnVector expected_measurement(3);

    // TODO: put predictions based on object velocity
    expected_measurement(1) = state(1);
    expected_measurement(2) = state(2);
    expected_measurement(2) = state(3);
    // ROS_INFO("1: state(1): %f, state(2): %f, state(3): %f", state(1), state(2), state(3));
    // ROS_INFO("1: meas(1): %f, meas(2): %f, meas(3): %f", measurement(1), measurement(2), measurement(3));


    Probability prb = _measNoise.ProbabilityGet(measurement - expected_measurement);

    return prb;
}
