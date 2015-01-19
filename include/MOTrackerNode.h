#ifndef __MOTRACKER_NODE__
#define __MOTRACKER_NODE__

#include <vector>
//structs
struct robot_t
{
  double x_,y_,theta_;
  double sec_;
};

struct ros_lrf_data_t
{
  int scan_num_;
  double resolution_;
  double first_angle_;
  std::vector<unsigned short>ranges_;
};

struct lrf_t
{
  int lrf_num_;
  std::vector<ros_lrf_data_t> lrf_data_;
};


#endif //__MOTRACKER_NODE__
