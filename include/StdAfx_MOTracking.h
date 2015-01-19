#pragma once

#ifdef _MSC_VER
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0501
#endif            
#define WIN32_LEAN_AND_MEAN
#pragma warning (push)
#pragma warning (disable: 4390)
#pragma warning (disable: 4819)
#pragma warning (disable: 4267)
#pragma warning (disable: 4244)
#define _USE_MATH_DEFINES
#endif

#include <math.h>
#include <cmath>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <list>
#include <map>
#include <boost/lexical_cast.hpp>
#include <boost/dynamic_bitset.hpp>

#include "MatrixFuncs.h"
#include "StreamException.h"
#include "NHUtil.h"
#include "PropertiedObject.h"
#include "Angle.h"

#include <boost/version.hpp>
#if (BOOST_VERSION < 105000)
#define TIME_UTC_ TIME_UTC
#endif


#ifdef _MSC_VER
#pragma warning (pop)
#endif
