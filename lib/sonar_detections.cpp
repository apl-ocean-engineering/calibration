
#include <fstream>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

#include "sonar_detections.h"

namespace camera_calibration {

  using namespace std;

  SonarDetection *SonarDetections::find( const std::string &timestamp )
  {
    for( unsigned int i = 0; i < _dets.size(); ++i ) {
      if( _dets[i]->timestamp().find( timestamp ) != string::npos ) return _dets[i];
    }

    return NULL;
  }

  void SonarDetections::load( const std::string &filename, bool imageAxes )
  {
    ifstream file( filename );

    string line;
    vector<string> tokens;
    while( std::getline( file, line ) ) {
      boost::split(tokens, line, boost::is_any_of(","));

      if( tokens.size() < 4 ) {
        LOG(INFO) << "Malformed sonar line: " << line;
        continue;
      }

      float x = atof( tokens[1].c_str() ),
      y = atof( tokens[2].c_str() ),
      z = atof( tokens[3].c_str() ),
      radius = atof( tokens[4].c_str() );

      if( imageAxes ) {
        float swap = y;
        y = -z;
        z = swap;
      }

      // Can only handle spheres right now
      _dets.push_back( new SonarSphereDetection( tokens[0], x,y,z,radius ) );

    }
  }


  //===================================================================

  ImageDetection *SonarSphereDetection::projectToImage( SonarImageWarper *warper )
  {
    cv::Vec2f center = warper->sonarToImage( cv::Vec3f( _x, _y, _z ) );

    // How to convert radius?
    float rad = 0;


    LOG(INFO) << "Mapped center from " << cv::Vec3f(_x,_y,_z) << " to " << center;
    LOG(INFO) << "Mapped radius from " << _radius << " to " << rad;

    return new ImageSphereDetection( _timestamp, center[0], center[1], rad );
  }


}
