#ifndef __DETECTION_DB_H__
#define __DETECTION_DB_H__

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include "detection.h"

#include <kchashdb.h>


using kyotocabinet::HashDB;

namespace AplCam {
  class DetectionSet;

  class DetectionDb {
    public:

      DetectionDb();
      DetectionDb( const string &dbFile, bool writer = false );

      ~DetectionDb();

      bool open( const string &dbFile, bool writer = false );
      bool open( const string &dbDir, const string &videoFile, bool writer = false );

      bool save( const int frame, const Detection &detection );
      bool save( const DetectionSet &detSet );

      bool has( const int frame );
      bool has( const string &key );

      bool update( const int frame, const Detection &detection );
      bool update( const string &key, const Detection &detection );

      Detection *load( const int frame );
      Detection *load( const string &key );

      kyotocabinet::BasicDB::Error error( void ) { return _db.error(); }

      int maxKey( void );

      static const std::string FrameToKey( const int frame );

      kyotocabinet::DB::Cursor *cursor( void );

    protected:



      HashDB _db;
      kyotocabinet::DB::Cursor *_cursor;

  };
}

#endif
