#include <cryptopp/sha.h>
#include <cryptopp/hex.h>
#include <cryptopp/files.h>

#include "image.h"
#include "file_utils.h"

using namespace std;


const string &Image::hash( void ) const
{
  if( _hash.empty() ) {
    _hash = fileHashSHA1( _fileName.string() );
  }

  return _hash;
}
