#include "db_object_filter/ObjectParticleFilter.h"

using namespace MatrixWrapper;
using namespace BFL;

ObjectParticleFilter::ObjectParticleFilter(MCPdf<ColumnVector> *prior, int resampleperiod, double resamplethreshold, int resamplescheme)
: BootstrapFilter<ColumnVector,ColumnVector>(prior, resampleperiod, resamplethreshold, resamplescheme)
{

}

vector<WeightedSample<ColumnVector> > ObjectParticleFilter::getNewSamples()
{
    return _new_samples;
}
