
#include "calibration_db.h"

namespace AplCam {


  using kyotocabinet::DB;

  //===========================================================================
  // CalibrationDb
  //===========================================================================

  bool CalibrationDb::has( const string &key )
  {
    return (_db.check( key ) != -1 ); 
  }

  bool CalibrationDb::save( const string &key, const CalibrationSerializer &ser )
  {
    string str;
    ser.serialize( str );
    if( !_db.set( key,  str ) ) return false;
    return true;
  }


  void CalibrationDb::findKeysStartingWith( const string &val, vector <string> &keys )
  {
    kyotocabinet::DB::Cursor *cur = _db.cursor();
    cur->jump();

    keys.clear();
    string thisKey;
    int len = val.size();
    while( cur->get_key( &thisKey, true ) ) {
      if( thisKey.compare( 0, len, val ) == 0 ) keys.push_back( thisKey );
    }
  }


}
