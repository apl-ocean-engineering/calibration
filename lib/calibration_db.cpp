

#include "calibration_db.h"

namespace AplCam {


  //===========================================================================
  // CalibrationDb
  //===========================================================================

  bool CalibrationDb::save( const string &key, const CalibrationSerializer &ser )
  {
    string str;
    ser.serialize( str );
    if( !_db.set( key,  str ) ) return false;
    return true;
  }


}
