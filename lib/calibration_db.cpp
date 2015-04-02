

#include "calibration_db.h"

namespace AplCam {


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


}
