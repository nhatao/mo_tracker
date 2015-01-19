#pragma once

#include "MOTracker.h"
#include "observer.h"
#include <ros/ros.h>

class CROSMessageSender : public CObserver
{
public:
  CROSMessageSender();
  virtual ~CROSMessageSender(void);

  void Init(const std::string &map_frame="/map", const std::string &tracker_topic="/human_track");
  virtual void Update(const CSubject& rSubject, const std::string &sMsg);

protected:

  std::string _sMapFrameName;
  ros::Publisher tracker_pub_;
  ros::NodeHandle nh_;
  std::string tracker_topic_;
  std::string map_frame_;

  //temp obstacleを吐く
  std::string obstacle_topic_;
  ros::Publisher obstacle_pub_;

  //temp Debug用
  std::string marker_topic_;
  ros::Publisher marker_pub_;


};

