#ifndef __SPECTRUM_H__
#define __SPECTRUM_H__

#include <opencv2/core.hpp>

using cv::Vec3f;
using cv::Vec3b;


class Spectrum {
public:
  Spectrum( int entries )
  : _l( entries ) {;}

  Vec3b operator()( int i, float V )
  {
    const float S = 1.0;
    float H = i * 360.0 / _l;

    float C = V*S;
    float Hp = H/60;
    float X = C* (1-fabs( fmod(Hp, 2) - 1 ));

    float m = V-C;

    if( (Hp >= 0) && (Hp < 1))
      return Vec3b( 255*(C+m), 255*(X+m), 0 );
    else if( (Hp >=1 ) && (Hp < 2))
      return Vec3b( 255*(X+m), 255*(C+m), 0 );
    else if( (Hp >= 2) && (Hp < 3))
      return Vec3b( 0, 255*(C+m),255*(X+m) );
    else if( (Hp >= 3) && (Hp < 4))
      return Vec3b( 0, 255*(X+m), 255*(C+m));
    else if( (Hp >= 4) && (Hp < 5))
      return Vec3b( 255*(X+m), 0, 255*(C+m) );
    else if( (Hp >= 5) && (Hp < 6))
      return Vec3b( 255*(C+m), 0, 255*(X+m) );

    return Vec3f(255,255,255);
  }

  int _l;
};

#endif
