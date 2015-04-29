
#include <iostream>

#include "calibrator.h"

#include "distortion_model.h"
#include "distortion_angular_polynomial.h"
#include "distortion_radial_polynomial.h"
using namespace Distortion;

#include "calibration_serializer.h"
#include "calibration_db.h"

namespace AplCam {

  using namespace std;

  void Calibrator::run( void )
  {

    Distortion::ImagePointsVecVec imagePoints;
    Distortion::ObjectPointsVecVec objectPoints;


    cout << "Have detection set " << _detSet.name() << " with " << _detSet.size() << " points" << endl;

    int count = _detSet.imageObjectPoints( imagePoints, objectPoints );


    cout << "Using " << count << " points from " << imagePoints.size() << " images" << endl;

    if( imagePoints.size() < 3 ) {
      cerr << "Not enough images.  Stopping." << endl;
      exit(-1);
    }


    _board = Board::load( _opts.boardPath(), _opts.boardName );
    _distModel = NULL;

    // Could/should farm this out to distortion..
    switch( _opts.calibType ) {
      case CalibrationOptsCommon::ANGULAR_POLYNOMIAL:
        _distModel = new Distortion::AngularPolynomial;
        break;
      case CalibrationOptsCommon::RADIAL_POLYNOMIAL:
        _distModel = new Distortion::RadialPolynomial;
        break;
    }

    if( !_distModel ) {
      cerr << "Something went wrong choosing a distortion model." << endl;
      exit(-1);
    }

    int flags =  _opts.calibFlags;
    _distModel->calibrate( objectPoints, imagePoints, 
        _imageSize, result, flags,
        cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 1000, DBL_EPSILON)  );  

    //  ///*|CV_CALIB_FIX_K3*/|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
    cout << "RMS error reported by calibrateCamera: " << result.rms << endl;
    cout << "Residual reported by calibrateCamera: " << result.residual << endl;
  }


  void Calibrator::saveDb( CalibrationDb &db, bool overwriteDb ) {

    CalibrationSerializer ser;

    ser.setCamera( _distModel )
      .setResult( &result )
      .setBoard( _board );

    if( db.has( _detSet.name() ) && !overwriteDb ) {
      cerr << "Already have result in db with key " << _detSet.name() << endl;
    } else {
      db.save( _detSet.name(), ser );
    }
  }

  void Calibrator::saveDb( const string &dbFile, bool overwriteDb ) {
    cout << "Writing to calibration db " << dbFile << endl;
    CalibrationDb db( dbFile );
    saveDb( db, overwriteDb );
  }



  void Calibrator::saveFile( const string &file ) {

    CalibrationSerializer ser;

    ser.setCamera( _distModel )
      .setResult( &result )
      .setBoard( _board );

    cout << "Writing calibration to " << file << endl;
    if( !ser.writeFile( file ) ) {
      cerr << "Error writing to opts.calibrationFile" << endl;;
    }
  }


  // For this special case, get a non-const DetectionSet
  void Calibrator::updateDetectionPoses( DetectionSet &dets )
  {
    // Try to assure they're the same DetectionSets...
    assert( dets.size() == _detSet.size() );

    cout << "Writing estimate board poses back to database." << endl;
    for( size_t i = 0; i < dets.size(); ++i ) {
      Detection &det( dets[i] );
      det.rot = result.rvecs[i];
      det.trans = result.tvecs[i];

    }
  }


}
