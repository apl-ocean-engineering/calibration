
#ifdef FOURIER_APPROACH

int normsSeconds( float start, float end, float **n )
{  return norms( floor( start * fps() ), ceil( end * fps() ), n ); }

int norms( int start, int end, float **n, int length = 0 )
{
  start = std::max( 0, std::min( frameCount(), start ) );
  end = std::max( 0, std::min( frameCount(), end ) );

  int window = end-start;
  int len = std::max( length, window );

  *n = (float *)valloc( len * sizeof(float) );

  seek( start );

  Mat prev;
  for( int at = start; at < end; ++at ) {
    Mat fullImage;
    capture >> fullImage;

    Mat timeCodeROI( fullImage, TimeCodeROI );

    if( prev.empty() ) {
      cv::cvtColor( timeCodeROI, prev, CV_BGR2GRAY );
      continue;
    }

    Mat curr;
    cv::cvtColor( timeCodeROI, curr, CV_BGR2GRAY );

    (*n)[2*at] = cv::norm( prev, curr, NORM_L2 ); 
    (*n)[2*at+1] = 0.0f;

    prev = curr;
  }

  // Zero pad the remainder
  memset( &(*n[window]), 0, (len - window) * sizeof(float) );

  return window;
}

// Should be window floor'ed to nearest power of two or somesuch
int windowLength = 128;
float *norms0, *norms1;
int count0 = video[0].norms( 0, windowLength, &norms0, windowLength*2 );
int count1 = video[1].norms( 0, windowLength, &norms1, windowLength*2 );

cout << "Got " << count0 << ", " << count1 << " norms" << endl;

ffts_plan_t *forward0 = ffts_init_1d_real( 2*windowLength, -1 );
ffts_plan_t *forward1 = ffts_init_1d_real( 2*windowLength, -1 );
ffts_plan_t *backward = ffts_init_1d_real( 2*windowLength, 1 );

// Real-to-complex transforms return (N/2 + 1) complex numbers
// But I've zero-padded the input to 2*windowLength
float fourierLength = (windowLength + 1);
float __attribute__ ((aligned(32))) *fourier0 = (float *)valloc( fourierLength * 2 * sizeof(float) );
float __attribute__ ((aligned(32))) *fourier1 = (float *)valloc( fourierLength * 2 * sizeof(float) );
float __attribute__ ((aligned(32))) *fourierRes = (float *)valloc( fourierLength * 2 * sizeof(float) );
float __attribute__ ((aligned(32))) *result = (float *)valloc( windowLength*2 * sizeof(float) );

ffts_execute( forward0, norms0, fourier0 );
ffts_execute( forward0, norms1, fourier1 );

// Convolve
float scale = 1.0 / fourierLength;
for( int i = 0; i < fourierLength; ++i ) {
  fourierRes[ 2*i ]   = (fourier0[ 2*i ]*fourier1[2*i]   + fourier0[2*i+1]*fourier1[2*i+1]) * scale;
  fourierRes[ 2*i+1 ] = (fourier0[ 2*i+1 ]*fourier1[2*i] - fourier0[2*i]*fourier1[2*i+1]) * scale;
}

ffts_execute( backward, fourierRes, result );

ofstream fst("correlation.txt");

float max = 0;
int maxIdx = -1;
for( int i = 0; i < 2*windowLength; ++i ) {
  if( maxIdx < 0 || result[i] >  max ) {
    maxIdx = i;
    max = result[i];
  }

  fst << i << " " << result[i] << endl;
}

fst.close();

cout << "Max occurs are index " << maxIdx << " value " << max << endl;

ffts_free( forward0 );
ffts_free( forward1 );
ffts_free( backward );

free( fourier0 );
free( fourier1 );
free( fourierRes );
free( result );


free( norms0 );
free( norms1 );

// Try a cross-correlation based approach (how will this deal with missing data?)


#endif
