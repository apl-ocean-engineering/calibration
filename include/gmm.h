#ifndef __GMM_H__
#define __GMM_H__

#include <vector>

#include <opencv2/core.hpp>

using cv::Mat;
using cv::Vec3d;

using std::vector;


/*
GMM - Gaussian Mixture Model
*/

class MaskedGMM;

class GMM
{
public:
  //static const int modelSize = 3/*mean*/ + 9/*covariance*/ + 1/*component weight*/;
  //static const int componentsCount = comp;

  GMM( unsigned int components );
  virtual ~GMM();

  double operator()( const Vec3d &color ) const;
  double logLikelihood( const Vec3d &color ) const;

  //double operator()( int ci, const Vec3d &color ) const;
  int whichComponent( const Vec3d &color ) const;

  float maxQ( const Vec3d &color ) const;
  float maxQat( const Vec3d &color, int &at ) const;

  void initLearning();
  void addSample( int ci, const Vec3d &color );
  void endLearning();

  //Vec3d mean( int ci ) const;

  struct Component {
    Component( void );

    double pdf( const Vec3d &color ) const;
    double weightedPdf( const Vec3d &color ) const;
    double logLikelihood( const Vec3d &color ) const;

    double uniformSq( const Vec3d &color ) const;

    void initLearning();
    void addSample( const Vec3d &color );
    void endLearning( int totalSampleCount );

    void calcInverseCovAndDeterm( void );

    double weight;
    Vec3d mean;
    double cov[3][3];

    double inverseCov[3][3];
    double covDeterm, invSqrtCovDeterm;

    double sum[3];
    double prod[3][3];
    int sampleCount;
  };

  unsigned int componentsCount( void ) const { return _components.size(); }
  const Component &operator[](unsigned int i ) const { return *_components[i]; }
  Component &operator[]( unsigned int i ) { return *_components[i]; }

protected:

  vector<Component *> _components;
  int totalSampleCount;
};


// Do this be composition rather than inheritence
class MaskedGMM
{
public:

  typedef unsigned char MaskType;

  MaskedGMM( unsigned int components );
  ~MaskedGMM( void );

  // double operator()( const Vec3d &color ) const;

  double logLikelihood( MaskType mask, const Vec3d &color ) const;

  //
  // //double operator()( int ci, const Vec3d &color ) const;

  int whichComponent( const Vec3d &color ) const;

  float maxQ( MaskType mask, const Vec3d &color ) const;
  float maxQat( MaskType mask, const Vec3d &color, int &at ) const;
  float maxQat( const Vec3d &color, int &at ) const                  { return _gmm.maxQat( color, at ); }

  void initLearning( void );
  void addSample( int ci, const Vec3d &color );
  void endLearning( void );


  void setMask( unsigned int idx, MaskType mask );
  MaskType maskAt( unsigned int idx ) const;

  unsigned int componentsCount( void ) const { return _componentsCount; }
  unsigned int componentsCount( MaskType mask ) const;


  // const Component &operator[](unsigned int i ) { return *_components[i]; }

protected:

  unsigned int _componentsCount;
  MaskType *_mask;

  GMM _gmm;

};

#endif
