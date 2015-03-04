

#include <stdlib.h>
#include <sys/stat.h>

#include <string>

using namespace std;


bool file_exists( const string &infile ) 
{
  struct stat buffer;   
  return (stat(infile.c_str(), &buffer) == 0 );
}

bool directory_exists( const string &infile ) 
{
  struct stat buffer;   
  return (stat(infile.c_str(), &buffer) == 0  && (buffer.st_mode & S_IFDIR));
}

void mkdir_p( const string &dir )
{
  char tmp[256];
  char *p = NULL;
  size_t len;

  snprintf(tmp, sizeof(tmp),"%s",dir.c_str() );
  len = strlen(tmp);

  bool finalSep = (tmp[len - 1] == '/');

  for(p = tmp + 1; *p; p++)
    if(*p == '/') {
      *p = 0;
      mkdir(tmp, S_IRWXU);
      *p = '/';
    }

  if( finalSep ) {
    tmp[len-1] = 0;
    mkdir(tmp, S_IRWXU);
    tmp[len-1] = '/';
  }
}
