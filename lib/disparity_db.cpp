
#include "disparity_db.h"

using namespace kyotocabinet;
using namespace std;
using namespace cv;

//========================================================================

const string SequentialDb::FpsKey = "fps";

SequentialDb::SequentialDb( void )
: _db(), _isOpen( false )
{;}

SequentialDb::SequentialDb( const string &filename, bool writable )
: _db(), _isOpen( false )
{
  open( filename );
}

bool SequentialDb::open( const string &filename, bool writable )
{
  uint flags = writable ? (BasicDB::OWRITER | BasicDB::OCREATE) :
  (BasicDB::OREADER | BasicDB::ONOLOCK );
  _isOpen = _db.open( filename, flags );

  return _isOpen;
}

void SequentialDb::close( void )
{
  if( isOpen() ) {
    _db.close();
    _isOpen = false;
  }
}

bool SequentialDb::setFps( double fps )
{
  if( !isOpen() ) return false;

  char fpsStr[16];
  snprintf( fpsStr, 15, "%15.12f", fps );
  return _db.set( FpsKey, fpsStr );
}


//========================================================================

const string DisparityMatDb::DisparityTagName = "disparity";

DisparityMatDb::DisparityMatDb( void )
: SequentialDb()
{;}

DisparityMatDb::DisparityMatDb( const string &filename, bool writable )
:SequentialDb( filename, writable )
{;}

bool DisparityMatDb::save( int frame, const Mat &disp )
{
  if( !isOpen() ) return false;
  char frameStr[16];
  snprintf( frameStr, 15, "%d", frame );

  cv::FileStorage st( "foo.yml.gz", FileStorage::WRITE | FileStorage::MEMORY );
  st << DisparityTagName << disp;

  return _db.set( frameStr, st.releaseAndGetString() );
}

//========================================================================

PointCloudDb::PointCloudDb( void )
: SequentialDb()
{;}

PointCloudDb::PointCloudDb( const string &filename, bool writable )
:SequentialDb( filename, writable )
{;}


bool PointCloudDb::save( int frame, const Mat &disp )
{
  return false;
}
