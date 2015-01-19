#include "StdAfx_MOTracking.h"
#include "AngleIDRange.h"

using namespace std;

CAngleIDRange::CAngleIDRange(void)
{
}


CAngleIDRange::~CAngleIDRange(void)
{
}



void TestAngleIDRange() {

  boost::numeric::interval<int> I1(10, 100);
  boost::numeric::interval<int> I2(120, 200);

  auto I3 = I1 + I2;


  cout << I3.lower() << " " << I3.upper() << endl;

}


void Test1() {

  typedef boost::numeric::interval_lib::checking_base<int> P2;
  typedef boost::numeric::interval<int, 
                                   boost::numeric::interval_lib::policies
                                     <boost::numeric::interval_lib::rounded_math<int>,
                                      P2> > RangeInterval;
  using namespace boost::numeric;

  RangeInterval d2 = hull(-1, 0);

  cout << d2.lower() << " " << d2.upper() << endl;
  cout << boolalpha;
  cout << empty(d2) << endl;




#if 0
  interval<int> d = hull(10, 25);

  cout << "1: " << boost::numeric::in(10, d) << endl;
  cout << "2: " << boost::numeric::in(11, d) << endl;


  typedef boost::numeric::interval_lib::checking_base<int> P2;
  typedef boost::numeric::interval<int, 
                                   boost::numeric::interval_lib::policies
                                     <boost::numeric::interval_lib::rounded_math<int>,
                                      P2> > RangeInterval;

  RangeInterval da = RangeInterval::empty();

  cout << "3: " << boost::numeric::in(10, da) << endl;
  cout << "4: " << boost::numeric::in(11, da) << endl;


  return;

  {
    interval<int> f = hull(8,30);
    cout << "sub : " << subset(f, d) << endl;
    cout << "sub : " << subset(d, f) << endl;
    cout << "psub: " << proper_subset(f, d) << endl;
    cout << "psub: " << proper_subset(d, f) << endl;
  }
  {
    interval<int> f = hull(10,30);
    cout << "sub : " << subset(f, d) << endl;
    cout << "sub : " << subset(d, f) << endl;
    cout << "psub: " << proper_subset(f, d) << endl;
    cout << "psub: " << proper_subset(d, f) << endl;
  }


  interval<int> f2 = hull(10,25);
  cout << subset(f2, d) << endl;
  cout << proper_subset(f2, d) << endl;
  cout << equal(f2, d) << endl;



  interval<int> d2 = hull(5, 18);
  interval<int> d3 = hull(20, 23);
  interval<int> d4 = hull(-4, 6);
  interval<int> d5 = hull(9, 32);

  vector<interval<int> > v1;
  v1.push_back(d2);
  v1.push_back(d3);
  v1.push_back(d4);
  v1.push_back(d5);

  cout << "PF: "  << d.lower() << ":" << d.upper() << endl;
  for (size_t i=0; i<v1.size(); ++i) {
    const auto &r1 = v1[i];
    cout << "Target: " << r1.lower() << ":" << r1.upper() << endl;
    cout << boolalpha;

    cout << "subset:" << subset(d, r1) << endl;
    cout << "psubset:" << proper_subset(d, r1) << endl;
    cout << "overlap:" << overlap(d, r1) << endl;

    auto p = hull(d, r1);
    cout << "hoge: " <<  p.lower() << ":" << p.upper() << endl;
  }
#endif
  
}