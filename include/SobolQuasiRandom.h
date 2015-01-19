#pragma once

#include <algorithm>

//準乱数の生成
//eusからコピー 正直よくわかってない
const size_t SOBOLMAXBIT = 30;
const size_t SOBOLMAXDIM =  6;
class CSobolQuasiRandom {

public:
  CSobolQuasiRandom(){Init();}
  ~CSobolQuasiRandom(){}

  void Init() {
    unsigned long mdeg2[SOBOLMAXDIM]={1,2,3,3,4,4};
    unsigned long ip2[SOBOLMAXDIM]={0,1,1,2,1,4};
    unsigned long iv2[SOBOLMAXDIM*SOBOLMAXBIT+1]={1,1,1,1,1,1,3,1,3,3,1,1,5,7,7,3,3,5,15,11,5,15,13,9};
    Init(mdeg2, ip2, iv2);
  }
  void Init(unsigned long *pmdeg, unsigned long *pip, unsigned long* piv) {
    memcpy(mdeg, pmdeg, sizeof(unsigned long)*SOBOLMAXDIM);
    memcpy(ip,   pip,   sizeof(unsigned long)*SOBOLMAXDIM);
    memcpy(iv,   piv,   sizeof(unsigned long)*SOBOLMAXDIM*SOBOLMAXBIT+1);
    unsigned long i,j,k,l;
    unsigned long ipp;
    unsigned long *iu[SOBOLMAXBIT];
    for (k=0;k<SOBOLMAXDIM;k++) ix[k]=0;
    in=0;
    fac=1.0/(1L << SOBOLMAXBIT);
    for (j=0,k=0;j<SOBOLMAXBIT;j++,k+=SOBOLMAXDIM) iu[j] = &iv[k];
    for (k=0;k<SOBOLMAXDIM;k++) {
      for (j=0;j<mdeg[k];j++) {
        iu[j][k] <<= (SOBOLMAXBIT-j-1);
      }
      for (j=mdeg[k];j<SOBOLMAXBIT;j++) { //Use the recurrence to get other values.
        ipp=ip[k]; 
        i=iu[j-mdeg[k]][k];
        i ^= (i >> mdeg[k]);
        for (l=mdeg[k]-1;l>=1;l--) {
          if (ipp & 1) i ^= iu[j-l][k];
          ipp >>= 1;
        }
        iu[j][k]=i;
      }
    }
  }
  void GetRandom(size_t n, double *x) {
    size_t j,k;
    unsigned long im;
    im=in++;
    for (j=0;j<SOBOLMAXBIT;j++) { // Find the rightmost zero bit.
      if (!(im & 1)) break;
      im >>= 1;
    }
    im=(unsigned long)(j*SOBOLMAXDIM);
    for (k=0;k<(std::min)(n,SOBOLMAXDIM);k++) { // XOR the appropriate direction number into each component of the vector and convert to a floating number.
      ix[k] ^= iv[im+k];
      x[k]=ix[k]*fac;
    }
  }

private:

  unsigned long mdeg[SOBOLMAXDIM];
  unsigned long ip[SOBOLMAXDIM];
  double fac;
  unsigned long in;
  unsigned long ix[SOBOLMAXDIM+1];
  unsigned long iv[SOBOLMAXDIM*SOBOLMAXBIT+1];

};
