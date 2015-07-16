

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <iostream>

#include <boost/thread.hpp>

#include "sonar_calibration_solver.h"

using namespace std;

using Eigen::Vector6d;

struct SonarCalibrationError {
  SonarCalibrationError( const SonarCalibrationSolver::SonarCalDatum &datum )
  : _datum( datum )
  {;}

  template <typename T>
  bool operator()( const T* const pose,
    T* residuals) const
{
return pointProjectionError( pose, residuals );
}

  template <typename T>
  bool conicError( const T* const pose,
    T* residuals) const
{
return true;
}

  template <typename T>
  bool pointProjectionError( const T* const pose,
    T* residuals) const
    {
      // pose is a 6-vector representing the pose of the sonar origin in
      // the camera frame.
      //
      //    3 angles
      //    3 translations
      // pose[0,1,2] are an angle-axis rotation.
      //
      const T sp[3] = { T(_datum.sonar[0]), T(_datum.sonar[1]), T(_datum.sonar[2]) };
      T p[3];
      ceres::AngleAxisRotatePoint(pose, sp, p);
      p[0] += pose[3];
      p[1] += pose[4];
      p[2] += pose[5];

      // p is now control point from sonar in camera frame (given pose)
      // As we're working in normalized, undistorted coordinates, the
      // projection of the sonar point is trivial.

      const T predicted[2] = { p[0]/p[2], p[1]/p[2] };

      // The error is the difference between the predicted and observed position.
      residuals[0] = predicted[0] - T( _datum.img[0] );
      residuals[1] = predicted[1] - T( _datum.img[1] );
      return true;
    }

    const SonarCalibrationSolver::SonarCalDatum &_datum;
    //   Vector2f _vPoint;
    // float _vRadius;
    //   Vector3f _sPoint;
    // float _sRadius;

  };

  struct SonarCalibrationErrorFactory {
    SonarCalibrationErrorFactory( Vector6d &pose, ceres::LossFunction *lossF = NULL )
    : _pose( pose ), _lossFunc( lossF )
    {;}

    void add( ceres::Problem &problem, const SonarCalibrationSolver::SonarCalDatum &datum )
    {
      ceres::CostFunction *costFunction = (new ceres::AutoDiffCostFunction<SonarCalibrationError, 2, 6>(
        new SonarCalibrationError( datum ) ) );

        problem.AddResidualBlock( costFunction, _lossFunc, _pose.data() );
      }

      Vector6d &_pose;
      ceres::LossFunction *_lossFunc;
    };



    bool SonarCalibrationSolver::solve( const SonarCalData &data, Result &result )
    {

      ceres::LossFunction *lossFunc = new ceres::HuberLoss( 4.0 );
      LOG(INFO) << "Using Huber loss function";

      result.pose = Vector6d::Zero();

      SonarCalibrationErrorFactory factory(  result.pose, lossFunc );

      ceres::Problem problem;

      for( size_t i = 0; i < data.size(); ++i ) {
        factory.add( problem, data[i] );
      }

      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.max_num_iterations = 100;
      options.minimizer_progress_to_stdout = true;

      // This should be configurable by the end user
      options.num_threads = boost::thread::hardware_concurrency();

      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      std::cout << summary.FullReport() << "\n";

      // Now attempt to find scale using radii.
      vector<float> scales;

      for( size_t i = 0; i < data.size(); ++i ) {
        Vector3f son( data[i].sonar );
        float radial = sqrt( son.dot( son ));

        float sonScale = data[i].sonarRadius / radial;

        float scale = data[i].imgRadius / sonScale;

        LOG(INFO) << "Son radius: " << data[i].sonarRadius << " radial = " << radial << " sonScale = " << sonScale << " imgRadius = " << data[i].imgRadius << " scale = " << scale;

        scales.push_back( scale );
      }

      float sonAvg = 0;
      for( size_t j = 0; j < scales.size(); ++j ) sonAvg += scales[j];
      sonAvg /= scales.size();

LOG(INFO) << "Average scale: " << sonAvg;

      //result.scale( sonAvg );


      //  result.totalTime = summary.total_time_in_seconds;

      // N.b. the Ceres cost is 1/2 || f(x) ||^2
      //
      // Whereas we usually use the RMS reprojection error
      //
      // sqrt( 1/N  ||f(x)||^2
      //
      // So rms = sqrt( 2/N final_cost )
      //
      result.residual = summary.final_cost;
      result.good = summary.IsSolutionUsable();

      return true;
    }


    #ifdef false

    struct ReprojectionError {
      ReprojectionError(double obs_x, double obs_y, double world_x, double world_y )
      : observedX(obs_x), observedY(obs_y), worldX( world_x ), worldY( world_y ) {;}

      template <typename T>
      void txToCameraFrame( const T* const pose, T *p ) const
      {
        const T point[3] = { T( worldX ), T( worldY ), T( 0.0 ) };
        ceres::AngleAxisRotatePoint(pose, point, p);
        p[0] += pose[3];
        p[1] += pose[4];
        p[2] += pose[5];
      }

      template <typename T>
      bool projectAndComputeError(const T* const camera,
        const T* const alpha,
        const T* const pp,
        T* residuals) const
        {
          const T &fx = camera[0];
          const T &fy = camera[1];
          const T &cx = camera[2];
          const T &cy = camera[3];
          const T &xpp = pp[0],
          &ypp = pp[1];

          T predictedX = fx*(xpp + alpha[0]*ypp) + cx;
          T predictedY = fy* ypp                 + cy;

          // The error is the difference between the predicted and observed position.
          residuals[0] = predictedX - T(observedX);
          residuals[1] = predictedY - T(observedY);
          return true;
        }


        double observedX, observedY;
        double worldX, worldY;
      };
      bool CeresRadialPolynomial::doCalibrate(
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

          double camera[4] = { _fx, _fy, _cx, _cy };
          double alpha = _alpha;

          // Inherently fragile.  Store coeffs in a double array instead?
          //    double k12[2] = { _distCoeffs[0], _distCoeffs[1] },
          //           p12[2] = { _distCoeffs[2], _distCoeffs[3] },
          //           k3[1]  = { _distCoeffs[4] },
          //           k456[3] = { _distCoeffs[5], _distCoeffs[6], _distCoeffs[7] };

          if( ! (flags & CV_CALIB_RATIONAL_MODEL ) ) {
            _distCoeffs[5] = 0.0;
            _distCoeffs[6] = 0.0;
            _distCoeffs[7] = 0.0;
          } else {
            cout << "Using rational model (with k4-k6)" << endl;
          }

          if( flags & CV_CALIB_ZERO_TANGENT_DIST ) {
            _distCoeffs[2] = 0.0;
            _distCoeffs[3] = 0.0;
            cout << "Fixing tangential distortion to zero" << endl;
          }
        }


        #endif
