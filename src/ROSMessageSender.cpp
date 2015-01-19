#include "StdAfx_MOTracking.h"
#include "ROSMessageSender.h"
#include "mo_tracker/Tracker.h"
#include "mo_tracker/Trackers.h"
#include "mo_tracker/Obstacle.h"
#include "mo_tracker/Obstacles.h"
#undef DELETE
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

CROSMessageSender::CROSMessageSender(){}
CROSMessageSender::~CROSMessageSender(void){}

void CROSMessageSender::Init(const std::string &map_frame, const std::string &tracker_topic) {

  ros::NodeHandle n_private_("~");
  n_private_.param("map_frame",map_frame_,map_frame);
  n_private_.param("tracker_topic",tracker_topic_,tracker_topic);
  string obstacle_topic_default = "/moving_obstacle";
  n_private_.param("obstacle_topic",obstacle_topic_,obstacle_topic_default);
  string marker_topic_default = "/moving_obstacle_marker";
  n_private_.param("obstacle_marker_topic",marker_topic_,marker_topic_default);

  cout << "map_frame: " << map_frame_ << endl;
  cout << "tracker_topic: " << tracker_topic_ << endl;
  cout << "obstacle_topic: " << obstacle_topic_ << endl;

  tracker_pub_=nh_.advertise<mo_tracker::Trackers>(tracker_topic_,1);
  obstacle_pub_=nh_.advertise<mo_tracker::Obstacles>(obstacle_topic_,1);
}

void CROSMessageSender::Update(const CSubject& rSubject, const std::string &sMsg) {

  if (sMsg != "TrackerUpdated") {
    return;
  }
  try {
    const CMOTracker &rTracker = dynamic_cast<const CMOTracker&>(rSubject);
    CTrackerResult Result;
    rTracker.GetLatestResult(Result);
    mo_tracker::Trackers tracker_msg;
    int n=0;
    for (auto it1 = Result.first->_vpObjects.begin();
              it1 != Result.first->_vpObjects.end(); ++it1, ++n) 
    {
      const auto &pStatus = (*it1)->GetStatus();
      mo_tracker::Tracker tracker;
      double dx = pStatus->_vPos(0);
      double dy = pStatus->_vPos(1);
      double dVx = pStatus->_vVel(0);
      double dVy = pStatus->_vVel(1);
      tracker.id=(*it1)->GetID();
      tracker.position.x=dx * 1e-3; tracker.position.y=dy * 1e-3;
      tracker.velocity.x=dVx * 1e-3; tracker.velocity.y=dVy * 1e-3;
      tracker.height =  tracker.height;

      const auto *pEC = dynamic_cast<const SEllipseCylinderMovingObjectStatus*>(pStatus.get());
      if (pEC) {
        double dR1 = pEC->_dR1/1000;
        double dR2 = pEC->_dR2/1000;
        tracker.type_name = "ellipse_cylinder";
        tracker.param1 = dR1;
        tracker.param2 = dR2;
      }
      const auto *pCuboid = dynamic_cast<const SCuboidMovingObjectStatus*>(pStatus.get());
      if (pCuboid) {
        double dLen1 = pCuboid->_dLength/1000;
        double dWid1 = pCuboid->_dWidth/1000;
        tracker.type_name = "cuboid";
        tracker.param1 = dLen1;
        tracker.param2 = dWid1;
      }

      tracker_msg.tracker.push_back(tracker);
    }
    if (!tracker_msg.tracker.empty()) {
      tracker_msg.header.stamp = ros::Time::fromBoost(Result.second.front()->GetPosixTime());
      tracker_msg.header.frame_id=map_frame_;
      tracker_pub_.publish(tracker_msg);
    }

    mo_tracker::Obstacles obstacle_msg;
    for (auto it1 = Result.first->_vpObjects.begin();
              it1 != Result.first->_vpObjects.end(); ++it1, ++n) 
    {
      const auto &pStatus = (*it1)->GetStatus();
      double dx = pStatus->_vPos(0)/1000;
      double dy = pStatus->_vPos(1)/1000;
      double dVx = pStatus->_vVel(0)/1000;
      double dVy = pStatus->_vVel(1)/1000;

      mo_tracker::Obstacle obstacle;
      obstacle.position.resize(4);

      double dWidth = 0.5;
      double dLength = 0;
      const auto *pCuboid = dynamic_cast<const SCuboidMovingObjectStatus*>(pStatus.get());
      if (pCuboid) {
        double dLen1 = pCuboid->_dLength/1000;
        double dWid1 = pCuboid->_dWidth/1000;

        dWidth = min(dLen1, dWid1)/2+0.2;
        dLength = max(dLen1, dWid1)/2;
      }
        
      CAngle dVelAngle = atan2(dVy, dVx);
      double dx2 = dVx*5 + dLength*cos(dVelAngle) + dx;
      double dy2 = dVy*5 + dLength*sin(dVelAngle) + dy;
      dx -= dLength*cos(dVelAngle);
      dy -= dLength*sin(dVelAngle);

      obstacle.position[0].x = dx + dWidth*cos(dVelAngle+M_PI/2);
      obstacle.position[0].y = dy + dWidth*sin(dVelAngle+M_PI/2);
      obstacle.position[0].z = 0;
      obstacle.position[1].x = dx + dWidth*cos(dVelAngle-M_PI/2);
      obstacle.position[1].y = dy + dWidth*sin(dVelAngle-M_PI/2);
      obstacle.position[1].z = 0;
      obstacle.position[2].x = dx2 + dWidth*cos(dVelAngle-M_PI/2);
      obstacle.position[2].y = dy2 + dWidth*sin(dVelAngle-M_PI/2);
      obstacle.position[2].z = 0;
      obstacle.position[3].x = dx2 + dWidth*cos(dVelAngle+M_PI/2);
      obstacle.position[3].y = dy2 + dWidth*sin(dVelAngle+M_PI/2);
      obstacle.position[3].z = 0;

      obstacle_msg.obstacle.push_back(obstacle);
    }
    if (!obstacle_msg.obstacle.empty()) {
      obstacle_msg.header.stamp = ros::Time::fromBoost(Result.second.front()->GetPosixTime());
      obstacle_msg.header.frame_id=map_frame_;
      obstacle_pub_.publish(obstacle_msg);
    }
  }

  catch (std::exception &e) {
    cout << __FUNCTION__ << " something wrong: " << e.what() << endl;
  }





}
