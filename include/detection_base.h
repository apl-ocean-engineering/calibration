#ifndef __DETECTION_BASE_H__
#define __DETECTION_BASE_H__

#include <string>

class DetectionBase {
public:
  DetectionBase( const std::string &timestamp )
  : _timestamp( timestamp )
  {;}

  bool match( const std::string &key )
  {
    return (_timestamp.compare( key ) == 0);
  }

  const std::string &timestamp( void ) const { return _timestamp; }

protected:
  std::string _timestamp;
};

#endif
