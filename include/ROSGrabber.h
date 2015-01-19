#pragma once
#include "MOTracker.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/thread.hpp>
#include <tf/transform_listener.h>

//とりあえずLRF1個だけ

class CROSGrabber :
  public CLRFGrabber
{
public:
  CROSGrabber(void);
  virtual ~CROSGrabber(void);
  void SetWaitForTransform(bool b) {_bWaitForTransform = b;}
  bool GetWaitForTransform() const {return _bWaitForTransform;}

  virtual void Start();
  virtual void Stop();

  void ShowDebug();

protected:

  bool GetTransform(std_msgs::Header header);

  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Subscriber map_sub_;

  //frames for transform
  std::string map_frame_;
  std::string base_frame_;
  //publish or subscribe topics
  std::string scan_topic_;
  std::string map_topic_;
  std::string tracker_topic_;

  bool b_update_;
  bool b_map_callback_;
  tf::TransformListener listener_;


  void CallbackThread()
  {
    ros::spin();
  }
  void ScanCallback(const sensor_msgs::LaserScanConstPtr &scan);

  boost::shared_ptr<CCascadedCoords> _pTempCoords;
  double _dTempTime;

  int _nReceived;
//  boost::posix_time::ptime _First;

  bool _bWaitForTransform;

//  boost::shared_ptr<CLaserData> _pCurrentData;
//  boost::recursive_mutex _CurrentDataMutex;
};

