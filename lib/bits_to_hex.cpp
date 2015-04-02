
#include "bits_to_hex.h"

namespace AplCam {

  const string intsToHex( const vector< int > &bits ) 
  {
    // Brute force way to do it
    vector<bool> bitmask;
    int mx = 0;

    for( vector<int>::const_iterator itr = bits.begin(); itr != bits.end(); ++itr ) 
      mx = std::max( (*itr), mx );

    bitmask.resize( mx+1 );

    for( vector<int>::const_iterator itr = bits.begin(); itr != bits.end(); ++itr ) 
      bitmask[ *itr ] = true;

    return bitsToHex( bitmask );

  }

  const string bitsToHex( const vector< bool > &bits ) 
  {
    static char hex[] = { '0', '1', '2', '3', '4', '5', '6', '7',
      '8', '9' ,'A', 'B', 'C', 'D', 'E', 'F' };

    string rev("");
    uint8_t accum = 0;
    uint8_t bit = 0;
    for( size_t i = 0; i < bits.size(); ++i ) {
      if( i > 0 && (i % 4) == 0 ) {
        rev.push_back( hex[accum] );
        accum = 0;
      }

      if( bits[i] ) accum |= (1 << (i % 4));
    }

    // Reverse string
    string out;
    std::copy( rev.rbegin(), rev.rend(), back_inserter(out) );

    return out;
  }


}
