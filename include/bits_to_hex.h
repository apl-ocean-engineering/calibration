#ifndef __BITS_TO_HEX_H__
#define __BITS_TO_HEX_H__

#include <string>
#include <vector>
#include <algorithm>

namespace AplCam {

  using std::string;
  using std::vector;

  const string intsToHex( const vector< int > &bits );
  const string bitsToHex( const vector< bool > &bits ); 
}

#endif


