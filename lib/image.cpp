#include <cryptopp/sha.h>
#include <cryptopp/hex.h>
#include <cryptopp/files.h>

#include "image.h"

using namespace std;


const string &Image::hash( void ) const
{
  if( _hash.empty() ) {
    CryptoPP::SHA1 hash;
    CryptoPP::FileSource(_fileName.string().c_str(), true, 
        new CryptoPP::HashFilter( hash,
          new CryptoPP::HexEncoder( new CryptoPP::StringSink( _hash ), true ) ) );
  }

  return _hash;
}
