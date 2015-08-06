

static unsigned char countBits( unsigned char v )
{
  unsigned char c; // store the total here
  static const int S[] = {1, 2, 4}; // Magic Binary Numbers
  static const int B[] = {0x55, 0x33, 0x0F};

  c = v - ((v >> 1) & B[0]);
  c = ((c >> S[1]) & B[1]) + (c & B[1]);
  c = ((c >> S[2]) + c) & B[2];

  return c;
}
