
#include <libg3logger/g3logger.h>

#include <opencv2/highgui/highgui.hpp>

#include "input_queue.h"

using namespace std;
using namespace cv;

namespace camera_calibration {

  InputQueue::InputQueue( const vector<fs::path> &files )
    : _files(), _idx(-1)
  {
      std::copy( files.begin(), files.end(), std::back_inserter(_files) );
  }


  InputQueue::InputQueue( const vector<std::string> &files )
    : _files(), _idx(-1)
  {
      std::transform( files.begin(), files.end(), std::back_inserter(_files),
                      [](std::string s) -> fs::path { return fs::path(s); } );
  }


  bool InputQueue::nextFrame( cv::Mat &mat )
  {
    // For now, only handles images.  Eventually will also handle video(s)

    do {
      ++_idx;
      if( (unsigned int)_idx >= _files.size() ) return false;
    } while( !fs::exists(_files[_idx]) );

    LOG(DEBUG) << "Reading " << _files[_idx].string();

    mat = cv::imread( _files[_idx].string() );

    return true;
  }

  std::string InputQueue::frameName( void )
  {
    return _files[_idx].stem().string();
  }

}
