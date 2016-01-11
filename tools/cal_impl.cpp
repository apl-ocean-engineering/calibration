
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "cal_impl.h"

#include "AplCam/detection/detection.h"

using namespace cv;

#define ANNOTATION_OUTPUT_EXT ".jpg"


CalOpts::CalOpts()
  : inFiles(),
    detectionOutput(), boardPath()
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
  // TCLAP::SwitchArg retryUnregArg("r", "retry-unregistered", "Retry unregistered point", cmd, false );
  // TCLAP::SwitchArg ignoreCacheArg("i", "ignore-cache", "Ignore cached points", cmd, false );
  TCLAP::SwitchArg doDetectArg("", "detect", "Do detection", cmd, false );
  TCLAP::ValueArg<string> detectOutputArg("", "detections-io", "", false, "", "Filename", cmd );
  TCLAP::ValueArg<string> drawDetectionsArg("", "draw-detections", "", false, "", "Filename", cmd );

  TCLAP::SwitchArg doCalibrateArg("", "calibrate", "Do calibrate", cmd, false );

  TCLAP::ValueArg<string> boardPathArg("", "board", "", false, "", "Filename", cmd );

  TCLAP::UnlabeledMultiArg< std::string > inFilesArg( "in-files", "Input files", true, "File names", cmd );

  cmd.parse( argc, argv );

  doDetect = doDetectArg.getValue();
  detectionOutput = detectOutputArg.getValue();
  drawDetectionPath = drawDetectionsArg.getValue();

  boardPath = boardPathArg.getValue();

  vector< string > inputs = inFilesArg.getValue();
  for( vector<string>::const_iterator itr = inputs.begin(); itr != inputs.end(); ++itr )
    inFiles.push_back( *itr );

}

bool CalOpts::validateOpts()
{
  if( !drawDetectionPath.empty() && !is_directory(drawDetectionPath) ) {
    LOG(ERROR) << "Detection annotation directory \"" << drawDetectionPath.string() << "\" does not exist";
    return false;
  }

  if( !boardPath.empty() && !fs::exists(boardPath) ) {
    LOG(ERROR) << "Specified board description file \"" << boardPath.string() << "\" does not exist";
    return false;
  }

  return true;
}




//===================================================================
// InputQueue
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
  int flags =  opts.calibFlags();

  DistortionModel *distModel = DistortionModel::MakeDistortionModel( opts.calibType );

  if( !distModel ) {
    cerr << "Something went wrong making a distortion model." << endl;
    exit(-1);
  }

  CalibrationResult result;
  distModel->calibrate( objectPoints, imagePoints,
      imageSize, result, flags );
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

  if( _detectionIO == NULL ) {
    LOG(FATAL) << "Unable to create detection I/O " << _opts.detectionOutput;
  }

  return _detectionIO;

}

//===================================================================
// InputQueue
//===================================================================


InputQueue::InputQueue( const vector<fs::path> &files )
  : _files( files ), _idx(-1)
{;}

cv::Mat InputQueue::nextFrame( void )
{
  // For now, only handles images

  do {
    ++_idx;
    if( (unsigned int)_idx >= _files.size() ) return Mat();
  } while( !fs::exists(_files[_idx]) );

  return cv::imread( _files[_idx].string() );
}

std::string InputQueue::frameName( void )
{
  return _files[_idx].stem().string();
}
