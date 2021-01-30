#pragma once

#include <filter/bootstrapfilter.h>

using namespace BFL;

class ObjectParticleFilter : public BootstrapFilter<MatrixWrapper::ColumnVector,MatrixWrapper::ColumnVector>
{
public:
   ObjectParticleFilter(MCPdf<MatrixWrapper::ColumnVector> * prior,
                         int resampleperiod = 0,
                         double resamplethreshold = 0,
                         int resamplescheme = DEFAULT_RS);

   vector<WeightedSample<MatrixWrapper::ColumnVector> > getNewSamples();

};
