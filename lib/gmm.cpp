
#include <math.h>
#include <gsl/gsl_cdf.h>

#include "gmm.h"

using namespace cv;

#include "count_bits.h"

GMM::Component::Component( void )
: weight( 0.0 ), covDeterm(0.0), invSqrtCovDeterm(0.0), sampleCount(0)
{
  for( unsigned int i = 0; i < 3; ++i ) {
    mean[i] = 0.0;
    for( unsigned int j = 0; j < 3; ++j ) cov[i][j]  = 0.0;
  }
  initLearning();
}

double GMM::Component::pdf( const Vec3d &color ) const
{
  if( weight > 0 )
  {
    return invSqrtCovDeterm * exp(-0.5f*uniformSq(color));
  }

  return 0;
}

double GMM::Component::weightedPdf( const Vec3d &color ) const
{
  return weight * pdf( color );
}

double GMM::Component::logLikelihood( const Vec3d &color ) const
{
  return log( weight * invSqrtCovDeterm ) - 0.5 * uniformSq( color );
}

// For the normal distribution
//   e^( 1/2 * (x - mu)^T Cov^-1 (x - mu) / cov ^2 )
//
//  Calculates the (x - mu)^T Cov^-1 (x - mu) / cov ^2 term
//  Such that it is equivalent to a zero-mean, 1-covariance normal:
//
//   e^( 1/2 * uniformSq() )
double GMM::Component::uniformSq( const Vec3d &color ) const
{
  if( weight > 0 )
  {
    // CV_Assert( covDeterms[ci] > std::numeric_limits<double>::epsilon() );

    Vec3d diff( color - mean );
    // double* m = _mean + 3*ci;
    // diff[0] -= m[0]; diff[1] -= m[1]; diff[2] -= m[2];

    double mult = diff[0]*(diff[0]*inverseCov[0][0] + diff[1]*inverseCov[1][0] + diff[2]*inverseCov[2][0])
    + diff[1]*(diff[0]*inverseCov[0][1] + diff[1]*inverseCov[1][1] + diff[2]*inverseCov[2][1])
    + diff[2]*(diff[0]*inverseCov[0][2] + diff[1]*inverseCov[1][2] + diff[2]*inverseCov[2][2]);

    return mult;
  }

  return 0;
}


void GMM::Component::initLearning()
{
  sum[0] = sum[1] = sum[2] = 0;
  prod[0][0] = prod[0][1] = prod[0][2] = 0;
  prod[1][0] = prod[1][1] = prod[1][2] = 0;
  prod[2][0] = prod[2][1] = prod[2][2] = 0;
  sampleCount = 0;
}

void GMM::Component::addSample( const Vec3d &color )
{
  sum[0] += color[0]; sum[1] += color[1]; sum[2] += color[2];
  prod[0][0] += color[0]*color[0]; prod[0][1] += color[0]*color[1]; prod[0][2] += color[0]*color[2];
  prod[1][0] += color[1]*color[0]; prod[1][1] += color[1]*color[1]; prod[1][2] += color[1]*color[2];
  prod[2][0] += color[2]*color[0]; prod[2][1] += color[2]*color[1]; prod[2][2] += color[2]*color[2];
  sampleCount++;
}


void GMM::Component::endLearning( int totalSampleCount )
{
  const double variance = 0.01;

  const int n = sampleCount;

  if( n == 0 ) {
    weight = 0;
  } else {
    weight = (double)n/totalSampleCount;

    mean[0] = sum[0]/n;
    mean[1] = sum[1]/n;
    mean[2] = sum[2]/n;

    for( unsigned int i = 0; i < 3 ; ++i )
    for( unsigned int j = 0; j < 3; ++j )
    cov[i][j] = prod[i][j]/n - mean[i]*mean[j];

    // Holdover from the old code
    double *dcov = &(cov[0][0]);
    double dtrm = dcov[0]*(dcov[4]*dcov[8]-dcov[5]*dcov[7]) - dcov[1]*(dcov[3]*dcov[8]-dcov[5]*dcov[6]) + dcov[2]*(dcov[3]*dcov[7]-dcov[4]*dcov[6]);

    if( dtrm <= std::numeric_limits<double>::epsilon() )
    {
      // Adds the white noise to avoid singular covariance matrix.
      cov[0][0] += variance;
      cov[1][1] += variance;
      cov[2][2] += variance;
    }

    calcInverseCovAndDeterm();
  }

}


// Hm. By hand.  Better (faster?) left to Eigen?
void GMM::Component::calcInverseCovAndDeterm( void  )
{
  if( weight > 0 )
  {
    double *dcov = &(cov[0][0]);
    covDeterm = dcov[0]*(dcov[4]*dcov[8]-dcov[5]*dcov[7]) - dcov[1]*(dcov[3]*dcov[8]-dcov[5]*dcov[6]) + dcov[2]*(dcov[3]*dcov[7]-dcov[4]*dcov[6]);
    invSqrtCovDeterm = 1.0f / sqrt( covDeterm );

    CV_Assert( covDeterm > std::numeric_limits<double>::epsilon() );

    inverseCov[0][0] =  (dcov[4]*dcov[8] - dcov[5]*dcov[7]) / covDeterm;
    inverseCov[1][0] = -(dcov[3]*dcov[8] - dcov[5]*dcov[6]) / covDeterm;
    inverseCov[2][0] =  (dcov[3]*dcov[7] - dcov[4]*dcov[6]) / covDeterm;
    inverseCov[0][1] = -(dcov[1]*dcov[8] - dcov[2]*dcov[7]) / covDeterm;
    inverseCov[1][1] =  (dcov[0]*dcov[8] - dcov[2]*dcov[6]) / covDeterm;
    inverseCov[2][1] = -(dcov[0]*dcov[7] - dcov[1]*dcov[6]) / covDeterm;
    inverseCov[0][2] =  (dcov[1]*dcov[5] - dcov[2]*dcov[4]) / covDeterm;
    inverseCov[1][2] = -(dcov[0]*dcov[5] - dcov[2]*dcov[3]) / covDeterm;
    inverseCov[2][2] =  (dcov[0]*dcov[4] - dcov[1]*dcov[3]) / covDeterm;
  }
}

//===================================================================


GMM::GMM( unsigned int components )
{

  for( unsigned int i = 0; i < components; ++i ) { _components.push_back( new Component ); }
  // // if( _model.empty() )
  // // {
  //   _model.create( 1, _modelSize*componentsCount, CV_64FC1 );
  //   _model.setTo(Scalar(0));
  // // }
  // // else if( (_model.type() != CV_64FC1) || (_model.rows != 1) || (_model.cols != _modelSize*componentsCount) )
  // // CV_Error( CV_StsBadArg, "_model must have CV_64FC1 type, rows == 1 and cols == 13*componentsCount" );

  // coefs = _model.ptr<double>(0);
  // _mean = coefs + componentsCount;
  // cov = _mean + 3*componentsCount;
  //
  // for( int ci = 0; ci < componentsCount; ci++ )
  //   if( coefs[ci] > 0 ) calcInverseCovAndDeterm( ci );
}

GMM::~GMM( void )
{
  for( unsigned int i = 0; i < componentsCount(); ++i ) { delete _components[i]; }
}

double GMM::logLikelihood( const Vec3d &color ) const
{
  double ll = 0;
  for( int ci = 0; ci < componentsCount(); ci++ )
  ll += _components[ci]->logLikelihood(color );
  return ll;
}

// TODO:: Shouldn't this by multiplication, not summation?
double GMM::operator()( const Vec3d &color ) const
{
  double res = 0;
  for( int ci = 0; ci < componentsCount(); ci++ )
  res *= _components[ci]->weightedPdf(color );
  return res;
}


int GMM::whichComponent( const Vec3d &color ) const
{
  int k = 0;
  double max = 0;

  for( int ci = 0; ci < componentsCount(); ci++ )
  {
    double p = _components[ci]->pdf( color );
    if( p > max )
    {
      k = ci;
      max = p;
    }
  }
  return k;
}


float GMM::maxQat( const Vec3d &color, int &at ) const
{
  float max = -1;
  at = -1;

  // Want to calculate the percentage of randomly drawn points
  // which are more extreme than this point
  // (1 = all of them (e.g., this is the mean))
  // This is the chi-squared Q function

  for( int ci = 0; ci < componentsCount(); ci++ ) {

    float xSq = _components[ci]->uniformSq( color );
    float q = gsl_cdf_chisq_Q( xSq, 3 );

    if( q > max ) {
      at = ci;
      max = q;
    }
  }

  return max;
}

float GMM::maxQ( const Vec3d &color ) const
{
  int at;
  return maxQat(color, at);
}

// Vec3d GMM::mean( int ci ) const
// {
//   return _components[ci].mean;
// }


void GMM::initLearning()
{
  for( int ci = 0; ci < componentsCount(); ci++) _components[ci]->initLearning();
  totalSampleCount = 0;
}

void GMM::addSample( int ci, const Vec3d &color )
{
  _components[ci]->addSample( color );
  totalSampleCount++;
}


void GMM::endLearning()
{
  for( int ci = 0; ci < componentsCount(); ci++ )    _components[ci]->endLearning( totalSampleCount );
}


//===================================================================

MaskedGMM::MaskedGMM( unsigned int components )
: _componentsCount( components ), _mask( new MaskType[components] ), _gmm( components )
{
  memset( _mask, 0, components );
}

MaskedGMM::~MaskedGMM( void )
{
  delete _mask;
}

void MaskedGMM::setMask( unsigned int idx, MaskType mask )
{
  assert( idx < _componentsCount );
  assert( countBits(mask) == 1 );
  _mask[idx] = mask;
}

MaskedGMM::MaskType MaskedGMM::maskAt( unsigned int idx ) const
{
  assert( idx < _componentsCount );
  return _mask[idx];
}

int MaskedGMM::maxPdfAt( const Vec3d &color ) const
{
  return maxPdfAt( 0xFF, color );
}

int MaskedGMM::maxPdfAt( MaskType mask, const Vec3d &color ) const
{
  int k = 0;
  double max = 0;

  for( int ci = 0; ci < componentsCount(); ci++ )
    if( maskAt(ci) & mask ) {
      double p = _gmm[ci].pdf( color );
      if( p > max )
      {
        k = ci;
        max = p;
      }
    }

  return k;
}

double MaskedGMM::logLikelihood( MaskType mask, const Vec3d &color ) const
{
  double ll = 0.0;
  bool set = false;

  for( int ci = 0; ci < componentsCount(); ci++ )
    if( maskAt(ci) & mask ) {
      ll += _gmm[ci].logLikelihood( color );
      set = true;
    }

  return (set ? ll : NAN );
}

double MaskedGMM::maxLogLikelihood( MaskType mask, const Vec3d &color ) const
{
  double ll = NAN;

  for( int ci = 0; ci < componentsCount(); ci++ )
    if( maskAt(ci) & mask ) {
      double l = _gmm[ci].logLikelihood( color );
      if( (l > ll) || isnan(ll) ) ll = l;
    }

  return ll;
}

float MaskedGMM::maxQat( MaskType mask, const Vec3d &color, int &at ) const
{
  float max = 0;
  at = -1;

  // Want to calculate the percentage of randomly drawn points
  // which are more extreme than this point
  // (1 = all of them (e.g., this is the mean))
  // This is the chi-squared Q function

  for( int ci = 0; ci < componentsCount(); ci++ ) {

    if( !(maskAt(ci) & mask) ) continue;

    float xSq = _gmm[ci].uniformSq( color );
    float q = gsl_cdf_chisq_Q( xSq, 3 );

    if( q > max ) {
      at = ci;
      max = q;
    }
  }

  return max;
}

float MaskedGMM::maxQ( MaskType mask, const Vec3d &color ) const
{
  int at;
  return maxQat(mask, color, at);
}

unsigned int MaskedGMM::componentsCount( MaskType mask ) const
{
  unsigned int count = 0;
  for( unsigned int i = 0; i < _componentsCount; ++i )
    if( maskAt(i) & mask ) ++count;

  return count;
}

void MaskedGMM::initLearning( void )
{
  _gmm.initLearning();
}

void MaskedGMM::addSample( int ci, const Vec3d &color )
{
  _gmm.addSample( ci, color );
}

void MaskedGMM::endLearning( void )
{
  for( unsigned int i = 0; i < 8*sizeof(MaskType); ++i ) {
    MaskType mask = 0x01<<i;

    int sampleCount = 0;
    for( int ci = 0; ci < componentsCount(); ci++ )
      if( maskAt(ci) & mask ) sampleCount += _gmm[ci].sampleCount;

    for( int ci = 0; ci < componentsCount(); ci++ )
      if( maskAt(ci) & mask ) _gmm[ci].endLearning( sampleCount );
  }
}
