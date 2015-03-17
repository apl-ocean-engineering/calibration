
#include "distortion_model.h"

namespace Distortion {

  using namespace std;
  using namespace cv;

  // Maintain the notation that the entry at (0,1) is alpha * fx
  PinholeCamera::PinholeCamera( void )
    : _fx(1.0), _fy(1.0), _alpha(0.0), _cx(0.0), _cy(0.0)
  {;}

  PinholeCamera::PinholeCamera( const Matx33d &k )
    : _fx( k(0,0) ), _fy( k(1,1) ), _alpha( k(0,1)/_fx ), 
      _cx( k(0,2) ), _cy( k(1,2) )
  {;}

  PinholeCamera::PinholeCamera( const Mat &k )
  { Matx33d kMatx;
    k.convertTo( kMatx, CV_64F );
    setCamera( kMatx ); 
  }

  void PinholeCamera::setCamera( const Matx33d &k )
  {
    setCamera( k(0,0), k(1,1), k(0,2), k(1,2), k(0,1)/k(0,0) );
  }

  void PinholeCamera::setCamera( double fx, double fy, double cx, double cy, double alpha )
  {
    _fx = fx;
    _fy = fy;
    _cx = cx;
    _cy = cy;
    _alpha = alpha;
  }

  Matx33d PinholeCamera::matx( void ) const 
  {
    return Matx33d( _fx, _alpha*_fx, _cx, 0., _fy, _cy, 0., 0., 1. );
  }

  Mat PinholeCamera::mat( void ) const
  { return Mat(matx()); }

};

