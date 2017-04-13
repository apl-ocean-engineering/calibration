
#include <opencv2/highgui/highgui.hpp>

#include "input_queue.h"

using namespace std;
using namespace cv;

namespace CameraCalibration {

InputQueue::InputQueue( const vector<fs::path> &files )
  : _files( files ), _idx(-1)
{;}

cv::Mat InputQueue::nextFrame( void )
{
  // For now, only handles images.  Eventually will also handle video(s)

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

}
