#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

typedef boost::numeric::ublas::matrix<double,boost::numeric::ublas::row_major,
        boost::numeric::ublas::unbounded_array<double> > BoostMat;
typedef boost::numeric::ublas::vector<double,boost::numeric::ublas::unbounded_array<double> > BoostVec;
