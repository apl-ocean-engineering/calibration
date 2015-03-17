
#ifndef __DISTORTION_MODEL_H__
#define __DISTORTION_MODEL_H__

#include <opencv2/core/core.hpp>
#include <opencv2/core/affine.hpp>
#include <vector>

namespace Distortion {

  using std::vector;
  using cv::Size;
  using cv::Mat;
  using cv::Vec3f;

  using cv::Vec2d;
  using cv::Vec3d;
  using cv::Vec4d;
  using cv::Point3d;
  using cv::Point2d;

  // For later ... it's all done double precision for now.  Not necessary.

  typedef vector< Point3d > ObjectPointsVec;
  typedef vector< vector< Point3d > > ObjectPointsVecVec;
  typedef vector< Point2d > ImagePointsVec;
  typedef vector< vector< Point2d > > ImagePointsVecVec;

  class DistortionModel {

    public:

      DistortionModel( void )
      {;}

  };



  class Fisheye : public DistortionModel {
    public:

      enum{
        CALIB_USE_INTRINSIC_GUESS   = (1<<0),
        CALIB_RECOMPUTE_EXTRINSIC   = (1<<1),
        CALIB_CHECK_COND            = (1<<2),
        CALIB_FIX_SKEW              = (1<<3),
        CALIB_FIX_K1                = (1<<4),
        CALIB_FIX_K2                = (1<<5),
        CALIB_FIX_K3                = (1<<6),
        CALIB_FIX_K4                = (1<<7),
        CALIB_FIX_INTRINSIC         = (1<<8)
      };


      Fisheye( void );
      Fisheye( const Vec4d &distCoeffs );

      double calibrate( const ObjectPointsVecVec &objectPoints, 
          const ImagePointsVecVec &imagePoints, const Size& image_size,
          Mat &K,
          vector< Vec3d > &rvecs, 
          vector< Vec3d > &tvecs,
          int flags = 0, 
          cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  );

      void undistortPoints( const vector< Point2d > &distorted, 
          vector< Point2d > &undistorted, 
          const Mat &K, 
          const Mat &R = cv::Mat::eye(3,3,CV_64F), 
          const Mat &P = cv::Mat());

      void projectPoints( const ObjectPointsVec &objectPoints, ImagePointsVec &imagePoints, 
          const cv::Affine3d& affine,
          const Mat &K, double alpha, cv::OutputArray jacobian);

      void projectPoints( const ObjectPointsVec &objectPoints, ImagePointsVec &imagePoints, 
          const Vec3d &_rvec, const Vec3d &_tvec, 
          const Mat &_K, double alpha, cv::OutputArray jacobian);

      struct IntrinsicParams
      {
        Vec2d f;
        Vec2d c;
        Vec4d k;
        double alpha;
        std::vector<int> isEstimate;

        IntrinsicParams();
        IntrinsicParams(Vec2d f, Vec2d c, Vec4d k, double alpha = 0);
        IntrinsicParams operator+(const Mat& a);
        IntrinsicParams& operator =(const Mat& a);

        void init(const cv::Vec2d& f, const cv::Vec2d& c, const cv::Vec4d& k = Vec4d(0,0,0,0), const double& alpha = 0);
      };


    protected: 

      void calibrateExtrinsics( const ObjectPointsVecVec &objectPoints,
          const ImagePointsVecVec &imagePoints,
          const IntrinsicParams& param, const int check_cond,
          const double thresh_cond,
          vector< Vec3d > &omc, 
          vector< Vec3d > &Tc );

      void computeJacobians( const ObjectPointsVecVec &objectPoints, 
          const ImagePointsVecVec &imagePoints,
          const IntrinsicParams& param, 
          const vector< Vec3d > &omc, const vector< Vec3d > &Tc,
          const int check_cond, const double thresh_cond, 
          Mat& JJ2_inv, Mat& ex3);

      double estimateUncertainties( const ObjectPointsVecVec &objectPoints, 
          const ImagePointsVecVec &imagePoints,
          const IntrinsicParams& params, 
          const vector< Vec3d > &omc, 
          const vector< Vec3d > &Tc,
          IntrinsicParams& errors, 
          Vec2d& std_err, 
          double thresh_cond, int check_cond );

      void initExtrinsics(const ImagePointsVec& _imagePoints, const ObjectPointsVec& _objectPoints, 
          const IntrinsicParams& param, Mat& omckk, Mat& Tckk);

      Mat computeHomography(Mat m, Mat M);

      Mat normalizePixels(const ImagePointsVec& imagePoints, const IntrinsicParams& param);

      void computeExtrinsicRefine(const ImagePointsVec& imagePoints, const ObjectPointsVec& objectPoints, 
          Mat &rvec, Mat &tvec, 
          Mat &J, const int MaxIter,
          const IntrinsicParams& param, const double thresh_cond);

      // Internal functions for projection points given an IntrinsicParams
      void projectPoints(const ObjectPointsVec &objectPoints, ImagePointsVec &imagePoints,
          const Vec3d &_rvec, const Vec3d &_tvec,
          const IntrinsicParams& param, cv::OutputArray jacobian);

      // This was formerly in its own anonymous namespace.
      // Maybe it should still be there...
      void subMatrix(const Mat& src, Mat& dst, const vector<int>& cols, const vector<int>& rows);

      cv::Vec4d _distCoeffs;

  };

}


#endif
