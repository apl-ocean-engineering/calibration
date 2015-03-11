
#include <string>

#ifndef __FILE_UTILS_H__
#define __FILE_UTILS_H__

bool file_exists( const std::string &infile );
bool directory_exists( const std::string &infile );
void mkdir_p( const std::string &dir );


#endif
