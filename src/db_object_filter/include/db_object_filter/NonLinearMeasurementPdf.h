#pragma once

#include <pdf/conditionalpdf.h>
#include <pdf/gaussian.h>
#include <math.h>
#include <wrappers/rng/rng.h> // Wrapper around several rng libraries

#include "ros/ros.h"
#include "ros/console.h"

using namespace BFL;

/// Non Linear Conditional Gaussian
class NonlinearMeasurementPdf : public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
{
public:
    /// Constructor
    /**
    @param additiveNoise Pdf representing the additive Gaussian uncertainty
    */
    NonlinearMeasurementPdf( const Gaussian& measNoise);

    /// Destructor
    virtual ~NonlinearMeasurementPdf();

    // implement this virtual function for measurement model of a particle filter
    virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const;

private:
    Gaussian _measNoise;
};
