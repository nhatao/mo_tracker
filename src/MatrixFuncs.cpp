#include "StdAfx_MOTracking.h"

#include "StreamException.h"
#include <iostream>
#include <math.h>
#include <boost/numeric/ublas/io.hpp>
#include "MatrixFuncs.h"
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>

using namespace std;
using namespace boost::numeric::ublas;

static const unsigned int MAX_MATRIX_SIZE = 5000;

//逆行列を解く
//clapackは逆行列が解けないことが多いのでやめた

bool InverseMatrix2D(const BoostMat& rTarget, BoostMat& rResult) {

  rResult.resize(2,2);

  double d1 = rTarget(0,0)*rTarget(1,1)+rTarget(1,0)*rTarget(0,1);
  if (d1 == 0) {
    ESStreamException e;
    e << __FUNCTION__ << " No Inverse matrix exists.";
    throw e;
  }
  rResult(0,0) =  rTarget(1,1)/d1;
  rResult(1,0) = -rTarget(1,0)/d1;
  rResult(0,1) = -rTarget(0,1)/d1;
  rResult(1,1) =  rTarget(0,0)/d1;
  return true;
}

bool InverseMatrix(const BoostMat& rTarget, BoostMat& rResult) {

  if ( (rTarget.size2() != rTarget.size1()) )
  {
    ESStreamException e;
    e << "Matrix Size has Invalid Size: " << rTarget.size1() << "/" << rTarget.size2();
    throw e;
  }
  if (rTarget.size1() == 2) {
    return InverseMatrix2D(rTarget, rResult);
  }

  BoostMat A(rTarget);
  BoostMat B = identity_matrix<double>(A.size1());
  permutation_matrix<> pm(A.size1());
  lu_factorize(A,pm);
  lu_substitute(A,pm,B);
  rResult.resize(B.size1(), B.size2());
  rResult = B;
  return true;
}

double Determinant2x2(const BoostMat& rMat) {
  return
          rMat(0, 0) * rMat(1, 1)         
        - rMat(1, 0) * rMat(0, 1);
}

double Determinant3x3(const BoostMat& rMat) {
    return
          rMat(0, 0) * rMat(1, 1) * rMat(2, 2)
        + rMat(2, 0) * rMat(0, 1) * rMat(1, 2)
        + rMat(1, 0) * rMat(2, 1) * rMat(0, 2)
                            
        - rMat(2, 0) * rMat(1, 1) * rMat(0, 2)
        - rMat(0, 0) * rMat(2, 1) * rMat(1, 2)
        - rMat(1, 0) * rMat(0, 1) * rMat(2, 2);
}


double Determinant(const BoostMat& m)
{
    namespace ublas = boost::numeric::ublas;

    BOOST_UBLAS_CHECK(m.size1() == m.size2(), ublas::external_logic());
    if (m.size1() == 2) return Determinant2x2(m);
    if (m.size1() == 3) return Determinant3x3(m);

    BoostMat lu(m);
    ublas::permutation_matrix<> pm(m.size1());

    ublas::lu_factorize(lu, pm);

    double det(1);

    typedef ublas::permutation_matrix<>::size_type size_type;

    for (size_type i = 0; i < pm.size(); ++i) {
        det *= (i == pm(i)) ? +lu(i, i) : -lu(i, i);
    }

    return det;
}

