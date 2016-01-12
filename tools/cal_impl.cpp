
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "cal_impl.h"

#include "AplCam/detection/detection.h"

using namespace cv;

#define ANNOTATION_OUTPUT_EXT ".jpg"


CalOpts::CalOpts()
  : inFiles()
{;}


bool CalOpts::parseOpts( int argc, char **argv )
{

  try {
    TCLAP::CmdLine cmd("cal", ' ', "0.1" );
    doParseCmdLine( cmd, argc, argv );
  } catch( TCLAP::ArgException &e ) {
    LOG( ERROR ) << "error: " << e.error() << " for arg " << e.argId();
    return false;
  }

  return validateOpts();
}

void CalOpts::doParseCmdLine( TCLAP::CmdLine &cmd, int argc, char **argv )
{
  TCLAP::ValueArg<string> boardPathArg("", "board", "", false, "", "Filename", cmd );

  // Detection opts
  TCLAP::SwitchArg doDetectArg("", "detect", "Do detection", cmd, false );
  TCLAP::ValueArg<string> detectOutputArg("", "detections-io", "", false, "", "Filename", cmd );
  TCLAP::ValueArg<string> drawDetectionsArg("", "draw-detections", "", false, "", "Filename", cmd );

  // Calibration opts
  TCLAP::SwitchArg doCalibrateArg("", "calibrate", "Do calibrate", cmd, false );
  TCLAP::ValueArg<string> distortionModelArg("", "distortion-model", "", false, "", "{radial|angular}", cmd );

  TCLAP::UnlabeledMultiArg< std::string > inFilesArg( "in-files", "Input files", true, "File names", cmd );

  cmd.parse( argc, argv );

  boardPath = boardPathArg.getValue();

  doDetect = doDetectArg.getValue();
  detectionOutput = detectOutputArg.getValue();
  drawDetectionPath = drawDetectionsArg.getValue();

  doCalibrate = doCalibrateArg.getValue();
  distortionModel = DistortionModel::ParseDistortionModel( distortionModelArg.getValue() );

  vector< string > inputs = inFilesArg.getValue();
  for( vector<string>::const_iterator itr = inputs.begin(); itr != inputs.end(); ++itr )
    inFiles.push_back( *itr );

}

bool CalOpts::validateOpts()
{
  if( !boardPath.empty() && !fs::exists(boardPath) ) {
    LOG(ERROR) << "Specified board description file \"" << boardPath.string() << "\" does not exist";
    return false;
  }

  if( !drawDetectionPath.empty() && !is_directory(drawDetectionPath) ) {
    LOG(ERROR) << "Detection annotation directory \"" << drawDetectionPath.string() << "\" does not exist";
    return false;
  }

  if( distortionModel == DistortionModel::CALIBRATION_NONE ) {
    LOG(ERROR) << "Unknown distorion model";
    return false;
  }

  return true;
}




//===================================================================
// Cal
//===================================================================

Cal::Cal( CalOpts &opts )
  : _opts( opts ), _inputQueue( _opts.inFiles ),
    _board( NULL ), _detectionIO( NULL )
{;}

Cal::~Cal()
{
  if( _board != NULL ) delete _board;
}

int Cal::run( void ) {
  if( _opts.doDetect ) doDetect();

  if( _opts.doCalibrate ) doCalibrate();

  return 0;
}

//--------------------------------------------------------------
// Detection functions

void Cal::doDetect( void )
{
  Mat img, imgGray;
  Detection *detection;

  detectionIO();

  while( !(img = _inputQueue.nextFrame()).empty() ) {

    cvtColor(img, imgGray, COLOR_BGR2GRAY);

    detection = board()->detectPattern( imgGray );

    if( !detection || !detection->good() ) {
      LOG(INFO) << "No detections for frame " << _inputQueue.frameName();
      continue;
    }

    LOG(INFO) << "  Found calibration pattern with " << detection->size() << " points";

    detectionIO()->save( _inputQueue.frameName(), detection );

    if( !_opts.drawDetectionPath.empty() )  drawDetection( img, detection );

  }
}

void Cal::drawDetection( const cv::Mat &img, Detection *detection )
{
  fs::path filename( _opts.drawDetectionPath );
  filename /= _inputQueue.frameName();
  filename.replace_extension( ANNOTATION_OUTPUT_EXT );

  Mat out;
  img.copyTo( out );
  board()->draw( out, detection );
  imwrite( filename.c_str(), out );
}

//--------------------------------------------------------------
// Calibration functions
void Cal::doCalibrate( void )
{
  CalibrationResult result;
  // model()->calibrate( objectPoints, imagePoints,
  //     imageSize, result, flags );
}


//--------------------------------------------------------------
// Eager-load functions

Board *Cal::board( void )
{
  if( _board != NULL ) return _board;

  _board = Board::load( _opts.boardPath.string(), _opts.boardPath.stem().string() );

  if( _board == NULL ) {
    LOG(FATAL) << "Unable to load board file " << _opts.boardPath;
  }

  return _board;
}

DetectionIO *Cal::detectionIO( void )
{
  if( _detectionIO != NULL ) return _detectionIO;

  _detectionIO = DetectionIO::Create( _opts.detectionOutput );

  if( _detectionIO == NULL ) LOG(FATAL) << "Unable to create detection I/O " << _opts.detectionOutput;

  return _detectionIO;
}

DistortionModel *Cal::model( void )
{
  if( _model != NULL ) return _model;

  _model = DistortionModel::MakeDistortionModel( _opts.distortionModel );

  if( _model == NULL ) LOG(FATAL) << "Unable to create distortion model " << _opts.detectionOutput;

  return _model;
}
