#ifndef __GMM_H__
#define __GMM_H__

#include <opencv2/core.hpp>

using cv::Mat;
using cv::Vec3d;


/*
GMM - Gaussian Mixture Model
*/
template <unsigned int comp>
class GMM
{
public:
  static const int modelSize = 3/*mean*/ + 9/*covariance*/ + 1/*component weight*/;
  static const int componentsCount = comp;

  GMM( void );

  double operator()( const Vec3d color ) const;
  double operator()( int ci, const Vec3d color ) const;
  int whichComponent( const Vec3d color ) const;

double uniform( int ci, const Vec3d color ) const;

  float maxQ( const Vec3d color ) const;
  float maxQat( const Vec3d color, int &at ) const;

  void initLearning();
  void addSample( int ci, const Vec3d color );
  void endLearning();

private:

  void calcInverseCovAndDeterm( int ci );
  Mat _model;
  double* coefs;
  double* mean;
  double* cov;

  double inverseCovs[comp][3][3];
  double covDeterms[comp];

  double sums[comp][3];
  double prods[comp][3][3];
  int sampleCounts[comp];
  int totalSampleCount;
};

#include "gmm.impl.hpp"

#endif
