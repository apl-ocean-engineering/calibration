
#ifndef __DISTORTION_MODEL_H__
#define __DISTORTION_MODEL_H__

#include <opencv2/core/core.hpp>
#include <opencv2/core/affine.hpp>
#include <vector>

namespace Distortion {

  using std::vector;
  using cv::Size;
  using cv::Mat;

  using cv::Vec2d;
  using cv::Vec3d;
  using cv::Vec4d;
  using cv::Point3d;
  using cv::Point2d;

  using cv::Matx33d;

  // For later ... it's all done double precision for now.  Not necessary.

  typedef vector< Point3d > ObjectPointsVec;
  typedef vector< vector< Point3d > > ObjectPointsVecVec;
  typedef vector< Point2d > ImagePointsVec;
  typedef vector< vector< Point2d > > ImagePointsVecVec;

  typedef vector< Vec3d > RotVec, TransVec;

  class PinholeCamera {
    public:

      enum{
        CALIB_RECOMPUTE_EXTRINSIC   = (1<<0),
        CALIB_CHECK_COND            = (1<<1),
        CALIB_FIX_SKEW              = (1<<2),
        CALIB_FIX_INTRINSIC         = (1<<3)
      };
      static const int CALIB_FLAG_OFFSET = 4;


      PinholeCamera( void );
      PinholeCamera( const Matx33d &k );
      PinholeCamera( const Mat &k );

     virtual const std::string name( void ) const { return "pinhole"; }

      void setCamera( const Matx33d &k );
      void setCamera( double fx, double fy, double cx, double cy, double alpha = 1 );

      Vec2d image( const Vec2d &pt ) const;
      Vec2d unimage( const Vec2d &pt ) const;

      Matx33d matx( void ) const;
      Mat mat( void ) const;

      Vec2d  f( void ) const      { return Vec2d( _fx, _fy ); }
      double fx( void ) const     { return _fx; }
      double fy( void ) const     { return _fy; }
      Vec2d c( void ) const       { return Vec2d(_cx,_cy); }
      double cx( void ) const     { return _cx; }
      double cy( void ) const     { return _cy; }
      double alpha( void) const   { return _alpha; }


      virtual cv::FileStorage &write( cv::FileStorage &out ) const;

    protected:

      double _fx, _fy, _alpha, _cx, _cy;

  };

  class DistortionModel : public PinholeCamera {

    public:

      DistortionModel( void )
        : PinholeCamera() {;}

      DistortionModel( const Matx33d &cam )
        : PinholeCamera( cam ) {;}


      virtual void projectPoints( const ObjectPointsVec &objectPoints, ImagePointsVec &imagePoints, 
          const Vec3d &_rvec, const Vec3d &_tvec, 
          cv::OutputArray jacobian = cv::noArray()) const = 0;

      //-- Undistortion functions --
     void undistortPoints( const vector< Point2d > &distorted, 
          vector< Point2d > &undistorted, 
          const Mat &R = cv::Mat::eye(3,3,CV_64F), 
          const Mat &P = cv::Mat());

      virtual void initUndistortRectifyMap( const Mat &R, const Mat &P,
          const cv::Size& size, int m1type, Mat &map1, Mat &map2 );


      void undistortImage( const Mat &distorted, Mat &undistorted,
          const Mat &Knew, const Size& new_size);

    protected:

       virtual Vec2d undistort( const Vec2d &pw ) const = 0;
       virtual Vec2d distort( const Vec3d &w ) const = 0;
  };

}


#endif
