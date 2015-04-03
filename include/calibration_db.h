#ifndef __CALIBRATION_DB_H__
#define __CALIBRATION_DB_H__

#include <string>
#include <kchashdb.h>

#include "calibration_serializer.h"

namespace AplCam {

  using std::string;
  using kyotocabinet::HashDB;

  class CalibrationDb {
    public:
      CalibrationDb( const string &filename )
        : _db()
      {
        _isOpen = _db.open( filename );
      }

      kyotocabinet::BasicDB::Error error( void ) { return _db.error(); }
      kyotocabinet::DB::Cursor *cursor( void ) { return _db.cursor(); }

      bool isOpened( void ) const { return _isOpen; }

      bool save( const string &key, const CalibrationSerializer &ser );
      bool get( const string &key, string * value )
      { return _db.get( key, value ); }

      bool has( const string &key );

      void findKeysStartingWith( const string &val, vector< string > &keys );
      

    protected:
      HashDB _db;
      bool _isOpen;
  };



}

#endif
