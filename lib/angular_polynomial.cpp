//
// Refactoring the OpenCV distortion model, starting with the AngularPolynomial model.
// Based on a heavily hacked version of fisheye.cpp from OpenCV.

#include <math.h>

#include "distortion_angular_polynomial.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <iostream>
#include <iomanip>
using namespace std;


//static std::ostream& operator <<(std::ostream& stream, const Distortion::AngularPolynomial &p )
//{
//  stream << p.f()[0] << " " << p.f()[1] << ", " << p.c()[0] << " " << p.c()[1] << ", " << p.alpha() << ", " << p.distCoeffs()[0] << " " << p.distCoeffs()[1] << " " << p.distCoeffs()[2] << " " << p.distCoeffs()[3];
//  return stream;
//}

namespace Distortion {

  //// Used by projectPoints
  //struct  __attribute__((packed)) JacobianRow
  //{
  //  Vec2d df, dc;
  //  double dalpha;
  //  Vec4d dk;
  //  Vec3d dom, dT;
  //};

  using namespace cv;
  using namespace std;

  // Polynomial expansion from (pg 16): 
  // http://www.research.scea.com/research/pdfs/RGREENfastermath_GDC02.pdf
  //
const Vec4d AngularPolynomial::ZeroDistortion = Vec4d( 0.334961658, 0.118066350, 0.092151584, 0 );

  AngularPolynomial::AngularPolynomial( void )
    : DistortionModel(), _distCoeffs( ZeroDistortion )
  {;}

  AngularPolynomial::AngularPolynomial( const Vec4d &distCoeffs )
    : DistortionModel(), _distCoeffs( distCoeffs )
  {;}

  AngularPolynomial::AngularPolynomial( const Vec4d &distCoeffs, const Matx33d &cam )
    : DistortionModel( cam ), _distCoeffs( distCoeffs )
  {;}

  // Static version uses a reasonable estimate based on image size
  //AngularPolynomial AngularPolynomial::Calibrate( 
  //    const ObjectPointsVecVec &objectPoints, 
  //    const ImagePointsVecVec &imagePoints, 
  //    const Size& image_size,
  //    vector< Vec3d > &rvecs, 
  //    vector< Vec3d > &tvecs,
  //    int flags, 
  //    cv::TermCriteria criteria)
  //{
  //  AngularPolynomial fe( ZeroDistortion, Camera::InitialCameraEstimate( image_size ) );
  //  fe.calibrate( objectPoints, imagePoints, image_size, rvecs, tvecs, flags, criteria );
  //  return fe;
  //}



  // Ceres functor for solving calibration problem
  //  Based on the Bundler solver used in their examples
  struct CalibReprojectionError {
    CalibReprojectionError(double obs_x, double obs_y, double world_x, double world_y )
      : observedX(obs_x), observedY(obs_y), worldX( world_x ), worldY( world_y ) {}

    template <typename T>
      bool operator()(const T* const camera,
          const T* const alpha,
          const T* const pose, 
          T* residuals) const
      {
        // pose is a 6-vector
        //    3 angles
        //    3 translations
        //
        // alpha is a 1-vector (separate so it can be set Constant/Variable)
        //
        // camera i s 8-vector
        //    2 focal length
        //    2 camera center
        //    4 distortion params
        //
        // pose[0,1,2] are an angle-axis rotation.
        //
        T point[3] = { T( worldX ), T( worldY ), T( 0.0 ) };
        T p[3];
        ceres::AngleAxisRotatePoint(pose, point, p);
        p[0] += pose[3]; 
        p[1] += pose[4]; 
        p[2] += pose[5];

        T theta = atan2( sqrt( p[0]*p[0] + p[1]*p[1] ), p[2]  );
        T psi   = atan2( p[1], p[0] );

        const T &fx = camera[0];
        const T &fy = camera[1];
        const T &cx = camera[2];
        const T &cy = camera[3];
        const T &k1 = camera[4];
        const T &k2 = camera[5];
        const T &k3 = camera[6];
        const T &k4 = camera[7];

        T theta2 =  theta*theta;
        T theta4 = theta2*theta2;
        T theta6 = theta4*theta2;
        T theta8 = theta4*theta4;

        T thetaDist = theta * ( T(1) + k1*theta2 + k2 *theta4 + k3*theta6 + k4*theta8);

        T xdn = thetaDist * cos( psi ),
          ydn = thetaDist * sin( psi );

        T predictedX = fx*(xdn + alpha[0]*ydn) + cx;
        T predictedY = fy* ydn              + cy;

        // The error is the difference between the predicted and observed position.
        residuals[0] = predictedX - T(observedX);
        residuals[1] = predictedY - T(observedY);
        return true;
      }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double observed_x, const double observed_y, 
        const double world_x, const double world_y ) {
      return (new ceres::AutoDiffCostFunction<CalibReprojectionError, 2, 8, 1, 6>(
            new CalibReprojectionError(observed_x, observed_y, world_x, world_y)));
    }

    double observedX, observedY;
    double worldX, worldY;
  };

  bool AngularPolynomial::doCalibrate(
      const ObjectPointsVecVec &objectPoints, 
      const ImagePointsVecVec &imagePoints, 
      const Size& image_size,
      CalibrationResult &result,
      int flags, 
      cv::TermCriteria criteria)
  {

    // Check and see if the camera matrix has been initialized
    if( norm( matx(), Mat::eye(3,3,CV_64F) ) < 1e-9 )
      setCamera( Camera::InitialCameraEstimate( image_size ) );

    int totalPoints = 0;
    int goodImages = 0;

    for( size_t i = 0; i < objectPoints.size(); ++i )  {
      if( result.status[i] ) {
        ImagePointsVec undistorted =  unwarp( normalize( imagePoints[i] ) );

        // Found the approach provided by initExtrinsics to be more reliable (!)
        // will need to investigate why that is.
        bool pnpRes = solvePnP( objectPoints[i], undistorted, Mat::eye(3,3,CV_64F), Mat(), 
            result.rvecs[i], result.tvecs[i], false, CV_ITERATIVE );

        //cout << "Pnp: " << (pnpRes ? "" : "FAIL") << endl << result.rvecs[i] << endl << result.tvecs[i] << endl;
        //initExtrinsics( imagePoints[i], objectPoints[i], rvecs[i], tvecs[i] );
        //cout << "initExtrinsics: " << endl << rvecs[i] << endl << tvecs[i] << endl;

        if( !pnpRes ) { 
          result.status[i] = false; 
          continue; 
        }

        ++goodImages;
        totalPoints += objectPoints[i].size();
      }
    }

    cout << "From " << objectPoints.size() << " images, using " << totalPoints << " from " << goodImages << " images" << endl;

    double camera[9] = { _fx, _fy, _cx, _cy,
      _distCoeffs[0], _distCoeffs[1],
      _distCoeffs[2], _distCoeffs[3] };
    double alpha = _alpha;

    double *pose = new double[ goodImages * 6];

    ceres::Problem problem;
    for( size_t i = 0, idx = 0; i < objectPoints.size(); ++i ) {
      if( result.status[i] ) {

        double *p = &( pose[idx*6] );
        // Mildly awkward
        p[0] = result.rvecs[i][0];
        p[1] = result.rvecs[i][1];
        p[2] = result.rvecs[i][2];
        p[3] = result.tvecs[i][0];
        p[4] = result.tvecs[i][1];
        p[5] = result.tvecs[i][2];

        ++idx;

        for( size_t j = 0; j < imagePoints[i].size(); ++j ) {
          ceres::CostFunction *costFunction = CalibReprojectionError::Create( imagePoints[i][j][0], imagePoints[i][j][1],
              objectPoints[i][j][0], objectPoints[i][j][1] );
          problem.AddResidualBlock( costFunction, NULL, camera, &alpha, p );
        }
      }
    }

    if( flags & CALIB_FIX_SKEW ) problem.SetParameterBlockConstant( &alpha );

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations = criteria.maxCount;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    for( size_t i = 0, idx=0; i < objectPoints.size(); ++i ) {
      if( result.status[i] ) {
        result.rvecs[i] = Vec3d( &(pose[idx*6]) );
        result.tvecs[i] = Vec3d( &(pose[idx*6+3]) );
        ++idx;
      }
    }

    delete[] pose;

    result.totalTime = summary.total_time_in_seconds;
    result.success = summary.IsSolutionUsable();
    
    // N.b. the Ceres cost is 1/2 || f(x) ||^2
    //
    // Whereas we usually use the RMS reprojection error
    //
    // sqrt( 1/N  ||f(x)||^2
    //
    // So rms = sqrt( 2/N final_cost )
    //
    result.residual = summary.final_cost;
    result.numPoints = totalPoints;
    result.numImages = goodImages;

    set(camera, alpha);

//    //
//    for( size_t i = 0; i < objectPoints.size(); ++i ) {
//      if( !result.status[i] ) continue;
//
//      for( size_t j = 0; j < objectPoints[i].size(); ++j ) {
//        ObjectPoint  obj = objectPoints[i][j];
//        ImagePoint   img = imagePoints[i][j];
//
//        CalibReprojectionError costFunction( img[0], img[1], obj[0], obj[1] );
//        double ceresResiduals[2], 
//               p[6] = { result.rvecs[i][0], result.rvecs[i][1], result.rvecs[i][2], result.tvecs[i][0], result.tvecs[i][1], result.tvecs[i][2] };
//        costFunction.operator()<double>( camera, &alpha, p, ceresResiduals );
//
//        ImagePoint projPt;
//        projectPoint( obj, result.rvecs[i], result.tvecs[i], projPt );
//        ImagePoint diff = projPt - img;
//
//        //cout << ceresResiduals[0] << " " << ceresResiduals[1] << "       " << diff[0] << " " << diff[1] << endl;
//        double cErr = (ceresResiduals[0]*ceresResiduals[0]) + (ceresResiduals[1]*ceresResiduals[1]);
//        double pErr = (diff[0]*diff[0]) + (diff[1]*diff[1]);
//
//        cerr << setw(20) << cErr << "   " << pErr << "     " << (cErr - pErr) << endl;
//      }
//    }
//
/////



    result.rms = reprojectionError( objectPoints, imagePoints, result );

    cout << "Final camera: " << endl << matx() << endl;
    cout << "Final distortions: " << endl << _distCoeffs << endl;

    return true;
  }


  ImagePoint AngularPolynomial::warp( const ObjectPoint &w ) const
  {
    double theta = atan2( sqrt( w[0]*w[0] + w[1]*w[1] ), w[2] );
    double psi = atan2( w[1], w[0] );

    double theta2 = theta*theta, 
           theta4 = theta2*theta2, 
           theta6 = theta4*theta2, 
           theta8 = theta4*theta4;

    double theta_d = theta * (1 + _distCoeffs[0]*theta2 + _distCoeffs[1]*theta4 + _distCoeffs[2]*theta6 + _distCoeffs[3]*theta8);

    return Vec2f( theta_d*cos( psi ), theta_d*sin(psi) );
  }


  // Use hammer to kill mosquito
  //  Based on the Bundler solver used in their examples
  struct UndistortReprojError {
    UndistortReprojError( double obs_x, double obs_y, const Vec4d &k )
      : observedX(obs_x), observedY(obs_y), _k( k ) {;}

    double observedX, observedY;
    const Vec4d _k;

    template <typename T>
      bool operator()(const T* const p,
          T* residuals) const
      {
        // point is a 2-vector
        //T p[2] = { point[0], point[1] };

        // Important to use atan2?
        T theta = atan( sqrt( p[0]*p[0] + p[1]*p[1] )  );
        T psi   = atan2( p[1], p[0] );

        T theta2 =  theta*theta;
        T theta4 =  theta2*theta2;
        T theta6 =  theta4*theta2;
        T theta8 =  theta4*theta4;

        T thetaDist = theta * ( T(1) + _k[0]*theta2 + _k[1]*theta4 + _k[2]*theta6 + _k[3]*theta8);

        T xdn = thetaDist * cos( psi ),
          ydn = thetaDist * sin( psi );

        // The error is the difference between the predicted and observed position.
        residuals[0] = xdn - T(observedX);
        residuals[1] = ydn - T(observedY);
        return true;
      }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double observed_x, const double observed_y, 
        const Vec4d &k ) {
      return (new ceres::AutoDiffCostFunction<UndistortReprojError, 2, 2>(
            new UndistortReprojError(observed_x, observed_y, k )));
    }
  };


  ImagePointsVec AngularPolynomial::unwarp( const ImagePointsVec &pw ) const
  {
    int Np = pw.size();
    double *p = new double[ Np*2 ];

    ceres::Problem problem;
    for( int i = 0; i < Np; ++i ) {
      p[ i*2 ] = pw[i][0];
      p[ i*2 + 1 ] = pw[i][1];

      ceres::CostFunction *costFunction = UndistortReprojError::Create( pw[i][0], pw[i][1], _distCoeffs );
      problem.AddResidualBlock( costFunction, NULL, &(p[i*2]) );
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    //options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << summary.FullReport() << "\n";

    ImagePointsVec out( pw.size() );
    for( int i = 0; i < Np; ++i ) {
      out[i] = ImagePoint( p[i*2], p[i*2 + 1] );
      //cout << i << ": " << pw[i][0] << "," << pw[i][1] << "     " << out[i][0] << "," << out[i][1] << endl;
    }

    delete[] p;

    return out;


  }

  ImagePoint AngularPolynomial::unwarp( const ImagePoint &pw ) const
  {
    double p[2] = { pw[0], pw[1] };

    ceres::Problem problem;
    ceres::CostFunction *costFunction = UndistortReprojError::Create( pw[0], pw[1], _distCoeffs );
    problem.AddResidualBlock( costFunction, NULL, p );

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    //options.num_threads = 2;
    //options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << summary.FullReport() << "\n";
      
    ImagePoint out( p[0], p[1] );
    //cout << pw[0] << "," << pw[1] << "     " << out[0] << "," << out[1] << endl;

    return out;

    //double scale = 1.0;

    //double theta_d = sqrt(pw[0]*pw[0] + pw[1]*pw[1]);
    //if (theta_d > 1e-8)
    //{
    //  // compensate distortion iteratively
    //  double theta = theta_d;
    //  for(int j = 0; j < 10; j++ )
    //  {
    //    double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta6*theta2;
    //    theta = theta_d / (1 + _distCoeffs[0] * theta2 + _distCoeffs[1] * theta4 + _distCoeffs[2] * theta6 + _distCoeffs[3] * theta8);
    //  }

    //  scale = std::tan(theta) / theta_d;
    //}

    //Vec2f pu = pw * scale; //undistorted point
  }


  FileStorage &AngularPolynomial::write( FileStorage &out ) const
  {
    DistortionModel::write( out );
    out << "distortion_coefficients" << _distCoeffs;

    return out;
  }

  AngularPolynomial *AngularPolynomial::Load( cv::FileStorage &in )
  {
    Mat kmat;
    Vec4d dist;

    in["camera_matrix"] >> kmat;
    Matx33d k;
    kmat.convertTo( k, CV_64F );

    in["distortion_coefficients"] >> dist;

    return new AngularPolynomial( dist, k );

  }
}


