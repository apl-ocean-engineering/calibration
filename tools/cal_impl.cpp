
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "cal_impl.h"

#include "detection/detection.h"

using namespace cv;

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
  TCLAP::ValueArg<string> detectOutputArg("", "detection-file", "", false, "", "Filename", cmd );

  TCLAP::ValueArg<string> boardPathArg("", "board", "", false, "", "Filename", cmd );

  TCLAP::UnlabeledMultiArg< std::string > inFilesArg( "in-files", "Input files", true, "File names", cmd );

  cmd.parse( argc, argv );

  doDetect = doDetectArg.getValue();
  detectionOutput = detectOutputArg.getValue();

  boardPath = boardPathArg.getValue();

  vector< string > inputs = inFilesArg.getValue();
  for( vector<string>::const_iterator itr = inputs.begin(); itr != inputs.end(); ++itr )
    inFiles.push_back( *itr );

}

bool CalOpts::validateOpts()
{
  if( !boardPath.empty() && !fs::exists(boardPath) ) {
    LOG(ERROR) << "Specified board description file " << boardPath.string() << " does not exist";
    return false;
  }

  return true;
}




//===================================================================
// InputQueue
//===================================================================

Cal::Cal( CalOpts &opts )
  : _opts( opts ), _inputQueue( _opts.inFiles ), _board( NULL )
{;}

Cal::~Cal()
{
  if( _board != NULL ) delete _board;
}

int Cal::run( void ) {
  if( _opts.doDetect ) doDetect();

  return 0;
}

void Cal::doDetect( void )
{
  Mat img, imgGray;
  Detection *detection;

  while( !(img = _inputQueue.nextFrame()).empty() ) {

    cvtColor(img, imgGray, COLOR_BGR2GRAY);

    detection = board()->detectPattern( imgGray );

    if( !detection ) {
      LOG(INFO) << "No detections for frame " << _inputQueue.frameName();
      continue;
    }
    if( detection->good() ){
      LOG(INFO) << "  Found calibration pattern with " << detection->size() << " points";
    }
  }
}


// Eager-load board if needed
Board *Cal::board( void )
{
  if( _board != NULL ) return _board;

  _board = Board::load( _opts.boardPath.string(), _opts.boardPath.stem().string() );

  if( _board == NULL ) {
    LOG(FATAL) << "Unable to load board file " << _opts.boardPath;
  }

  return _board;
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
  return _files[_idx].string();
}
