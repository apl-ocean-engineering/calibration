
#ifndef __DISTORTION_FISHEYE_H__
#define __DISTORTION_FISHEYE_H__

#include "distortion_model.h"

namespace Distortion {


  class Fisheye : public DistortionModel {
    public:

      Fisheye( void );
      Fisheye( const Vec4d &distCoeffs );
      Fisheye( const Vec4d &distCoeffs, const Matx33d &cam );

      static Fisheye Calibrate( const ObjectPointsVecVec &objectPoints, 
          const ImagePointsVecVec &imagePoints, const Size& image_size,
          vector< Vec3d > &rvecs, 
          vector< Vec3d > &tvecs,
          int flags = 0, 
          cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  );

      double calibrate( const ObjectPointsVecVec &objectPoints, 
          const ImagePointsVecVec &imagePoints, const Size& image_size,
          vector< Vec3d > &rvecs, 
          vector< Vec3d > &tvecs,
          int flags = 0, 
          cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  );



      void undistortPoints( const vector< Point2d > &distorted, 
          vector< Point2d > &undistorted, 
          const Mat &R = cv::Mat::eye(3,3,CV_64F), 
          const Mat &P = cv::Mat());

      void projectPoints( const ObjectPointsVec &objectPoints, ImagePointsVec &imagePoints, 
          const cv::Affine3d& affine,
          cv::OutputArray jacobian = cv::noArray()) const;

      virtual void projectPoints( const ObjectPointsVec &objectPoints, ImagePointsVec &imagePoints, 
          const Vec3d &_rvec, const Vec3d &_tvec, 
          cv::OutputArray jacobian = cv::noArray() ) const;

      virtual void initUndistortRectifyMap( const Mat &R, const Mat &P,
          const cv::Size& size, int m1type, Mat &map1, Mat &map2 ) {;}


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

  static Matx33d InitialCameraEstimate( const Size &image_size );

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

      void normalizePixels(const ImagePointsVec& imagePoints, const IntrinsicParams& param, Mat &normalized);

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
