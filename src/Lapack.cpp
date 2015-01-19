#include "StdAfx_MOTracking.h"
#include "Lapack.h"

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
//#include <boost/numeric/ublas/io.hpp>


boost::mutex g_LapackMutex;

extern "C"{
  //windef.hが先にincludeされていると変なことになるため
#ifdef _MSC_VER
  #undef min
  #undef max
#endif
  #include <INCLUDE/f2c.h>
  #include <INCLUDE/clapack.h>
}

int dgeev2_(char *jobvl, char *jobvr, long int *n, double *
  a, long int *lda, double *wr, double *wi, double *vl, 
  long int *ldvl, double *vr, long int *ldvr, double *work, 
  long int *lwork, long int *info) {
  return dgeev_(jobvl, jobvr, n, a, lda, wr, wi, vl, ldvl, vr, ldvr, work, lwork, info);
}

int dgesvd2_(char *jobu, char *jobvt, long int *m, long int *n, 
  double *a, long int *lda, double *s, double *u, long int *
  ldu, double *vt, long int *ldvt, double *work, long int *lwork, 
  long int *info) {

  return dgesvd_(jobu, jobvt, m, n, a, lda, s, u, ldu, vt, ldvt, work, lwork, info);
}
