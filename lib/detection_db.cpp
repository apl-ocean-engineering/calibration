
#include "detection_db.h"
#include "detection_set.h"

#include "file_utils.h"

using namespace std;
using namespace cv;

namespace AplCam {

  //============================================================================
  //  DetectionDb
  //============================================================================

  DetectionDb::DetectionDb( void )
    : _db(), _cursor(NULL)
  {;}

  DetectionDb::DetectionDb( const string &dbFile, bool writer )
    : _db(), _cursor(NULL)
  {
    open( dbFile, writer );
  }



  DetectionDb::~DetectionDb()
  {
    _db.close();
    // if( _cursor ) delete _cursor;
  }

  bool DetectionDb::open( const string &dbDir, const string &videoFile, bool writer )
  {
    string dbfile = dbDir + "/" + fileHashSHA1( videoFile ) + ".kch";
    return open( dbfile, writer );
  }

  bool DetectionDb::open( const string &dbFile, bool writer )
  {
    int flags = (writer==true) ? (HashDB::OWRITER | HashDB::OCREATE) : (HashDB::OREADER);
    if( !_db.open( dbFile, flags ) ) return false;
    return true;
  }

  bool DetectionDb::save( const int frame, const Detection &detection )
  {
    string str;
    detection.serialize( str );
    if( !_db.set( FrameToKey( frame ),  str ) ) return false;
    return true;
  }

  bool DetectionDb::save( const DetectionSet &detSet )
  {
    size_t end = detSet.size();
    for( size_t i = 0; i < end; ++i ) save( detSet[i].frame, detSet[i] );

    return true;
  }

  bool DetectionDb::has( const int frame )
  {
    return has( FrameToKey( frame ) );
  }

  bool DetectionDb::has( const string &key )
  {
    return (_db.check( key ) != -1 ); 
  }

  bool DetectionDb::update( const int frame, const Detection &detection )
  {
    return update( FrameToKey( frame ), detection );
  }


  bool DetectionDb::update( const string &key, const Detection &detection )
  {
    string str;
    detection.serialize( str );
    if( !_db.set( key,  str ) ) return false;
    return true;
  }

  Detection *DetectionDb::load( const int frame )
  {
    return load( FrameToKey( frame ) );
  }

  Detection *DetectionDb::load( const string &key )
  {
    if( ! has(key) ) return NULL;

    string value;
    _db.get( key, &value );
    return Detection::unserialize( value );
  }

  kyotocabinet::DB::Cursor *DetectionDb::cursor( void )
  {
    if( _cursor == NULL ) {
      _cursor = _db.cursor();
      _cursor->jump();
    }

    return _cursor;
  }

  struct MaxFinder : public kyotocabinet::DB::Visitor
  {
    MaxFinder( void )
      : _max(0) {;}

    int max( void ) const { return _max; };

    virtual const char *visit_full( const char *kbuf, size_t ksiz, const char *vbuf, size_t vsiz, size_t *sp )
    {
      _max = std::max( _max, (int)atoi( kbuf ) );
      return Visitor::NOP;
    }

    int _max;
  };

  int DetectionDb::maxKey( void )
  {
    MaxFinder maxFinder;

    // false == readonly
    _db.iterate( &maxFinder, false );

    return maxFinder.max();
  }

  const string DetectionDb::FrameToKey( const int frame )
  {
    const int strWidth = 20;
    char frameKey[strWidth];
    snprintf( frameKey, strWidth-1, "%d", frame );

    return string( frameKey );
  }

}
