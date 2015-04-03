#include <opencv2/core/core.hpp>
#include "types.h"

namespace AplCam {
  using namespace std;
  using namespace cv;

  static Vec4f TxReprojErrorsToVec( const ImagePoint &projPt, const ImagePoint &error )
  {
return Vec4f( projPt[0], projPt[1], error[0], error[1] );
  }

  void write(cv::FileStorage &fs, std::string a, const ReprojErrorsVec& v )
  {
    // Lazy approach for now
    int sz = v.projPoints.size();
    vector< Vec4f > out( sz );

    std::transform( v.projPoints.begin(), v.projPoints.end(), v.errors.begin(), out.begin(), 
        TxReprojErrorsToVec );

    write( fs, a, Mat(out) );
  }

  void write(cv::FileStorage &fs, std::string a, const ReprojErrorsVecVec &v )
  {
    vector< Vec4f > out;

    // Feeling lazy, this approach will be slower due to continual resizing of out..
    for( size_t j = 0; j < v.size(); ++j ) 
      std::transform( v[j].projPoints.begin(), v[j].projPoints.end(), v[j].errors.begin(), 
          back_inserter( out ), TxReprojErrorsToVec );


    write( fs, a, Mat(out) );
  }

}
