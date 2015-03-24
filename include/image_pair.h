
#ifndef __IMAGE_PAIR_H__
#define __IMAGE_PAIR_H__

#include "image.h"

namespace AplCam {

  class ImagePair
  {
    public:
      ImagePair( const Image &a, const Image &b )
        : _a(a), _b(b) {;}

      const Image &operator[](int i) const {
        switch(i) {
          case 0: return _a; break;
          case 1: return _b; break;
        }
      }

    private:
      Image _a, _b;
  };

}

#endif

