
#ifndef __DISTORTION_RADIAL_POLYNOMIAL_H__
#define __DISTORTION_RADIAL_POLYNOMIAL_H__

#include "distortion_model.h"

namespace cv {
  typedef Vec<double,5> Vec5d;
  typedef Vec<double,8> Vec8d;
}

namespace Distortion {

  using cv::Vec4d;
  using cv::Vec5d;
  using cv::Vec8d;

  class RadialPolynomial : public DistortionModel {
    public:


      RadialPolynomial( void );
      RadialPolynomial( const Vec8d &distCoeffs );
      RadialPolynomial( const Vec5d &distCoeffs );
      RadialPolynomial( const Vec4d &distCoeffs );
      RadialPolynomial( const Vec8d &distCoeffs, const Matx33d &cam );
      RadialPolynomial( const Vec5d &distCoeffs, const Matx33d &cam );
      RadialPolynomial( const Vec4d &distCoeffs, const Matx33d &cam );

      //void set(const cv::Vec2d& f, const cv::Vec2d& c, const double &alpha = 0, const cv::Vec4d& k = Vec4d(0,0,0,0) )
      //{
      //  setCamera( f[0], f[1], c[0], c[1], alpha );
      //  _distCoeffs = k;
      //}

      static const std::string Name( void ) { return "RadialPolynomial"; }
      virtual const std::string name( void ) const { return RadialPolynomial::Name(); }

      Vec8d distCoeffs( void ) const    { return _distCoeffs; }

      //static RadialPolynomial Calibrate( const ObjectPointsVecVec &objectPoints, 
      //    const ImagePointsVecVec &imagePoints, const Size& image_size,
      //    vector< Vec3d > &rvecs, 
      //    vector< Vec3d > &tvecs,
      //    int flags = 0, 
      //    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  );

       virtual void projectPoints( const ObjectPointsVec &objectPoints, 
          const Vec3d &_rvec, const Vec3d &_tvec, ImagePointsVec &imagePoints, 
          cv::OutputArray jacobian = cv::noArray() ) const;

      virtual cv::FileStorage &write( cv::FileStorage &out ) const;
      static RadialPolynomial *Load( cv::FileStorage &in );

    protected: 

      virtual bool doCalibrate( const ObjectPointsVecVec &objectPoints, 
          const ImagePointsVecVec &imagePoints, const Size& image_size,
          CalibrationResult &result,
          int flags = 0, 
          cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  );

       virtual ImagePoint undistort( const ImagePoint &pw ) const;
       virtual ImagePoint distort( const Vec3f &w ) const ;

       static const Vec8d InitialDistortionEstimate( void ) 
       { return Vec8d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); }

      cv::Vec8d _distCoeffs;

  };


}


#endif
