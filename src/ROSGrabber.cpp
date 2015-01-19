#include "StdAfx_MOTracking.h"
#include "ROSGrabber.h"
#include <std_msgs/Header.h>
#include <boost/date_time/posix_time/posix_time_io.hpp>

using namespace std;

//default frames name
const std::string MAP_FRAME="/map";
const std::string BASE_FRAME="/base_link";

//default topics name
//const std::string MAP_TOPIC="/amcl_map";
const std::string SCAN_TOPIC="/scan";
const std::string TRACKER_TOPIC="/human_track";

#ifdef _MSC_VER
#include <mmsystem.h>
#else 
#include <sys/time.h> 
#endif
extern void ScanCallback2(const sensor_msgs::LaserScanConstPtr &scan);

CROSGrabber::CROSGrabber(void) : 
  b_update_(false),
  b_map_callback_(false)
{
  ros::NodeHandle n_private_("~");
  n_private_.param("map_frame",map_frame_,MAP_FRAME);
  n_private_.param("base_frame",base_frame_,BASE_FRAME);
  n_private_.param("scan_topic",scan_topic_,SCAN_TOPIC);
  n_private_.param("tracker_topic",tracker_topic_,TRACKER_TOPIC);
  _nReceived = 0;
  scan_sub_=nh_.subscribe(scan_topic_,100,&CROSGrabber::ScanCallback,this);
  _bWaitForTransform = false;
}

void CROSGrabber::ShowDebug() {
  cout << "map_frame_:" << map_frame_ << endl;
  cout << "base_frame_:" << base_frame_ << endl;
  cout << "scan_topic_:" << scan_topic_ << endl;
  cout << "tracker_topic_:" << tracker_topic_ << endl;
  cout << "WaitForTransform: " << boolalpha << _bWaitForTransform << endl;
}

CROSGrabber::~CROSGrabber(void){}
void CROSGrabber::Start() {}
void CROSGrabber::Stop() {}


void CROSGrabber::ScanCallback(const sensor_msgs::LaserScanConstPtr &scan)
{
  if (GetTransform(scan->header)) {

    SLRFProperty prop;
    prop._nElemNum = (int)scan->ranges.size();
    prop._dReso = scan->angle_increment;
    prop._dFirstAngle = scan->angle_min;
    prop._dMaxRange = scan->range_max*1000;
    prop._dMinRange = scan->range_min*1000;
    prop._dTimeIncrement = scan->time_increment;
    prop._dScanTime = scan->scan_time;

    boost::shared_ptr<CLaserData> pData(new CLaserData(prop, scan->header.stamp.toBoost(), _pTempCoords));
    pData->Set2DData(scan->ranges.begin(), scan->ranges.end(), 1000);
    std::vector<boost::shared_ptr<const CLaserData>> v1;
    v1.push_back(pData);
    SetLatestResult(v1);

    if (_nReceived == 0) {
      cout << "PointNum: " << prop._nElemNum << endl;
      cout << "Resolution: " << prop._dReso.get_deg() << endl;
      cout << "FirstAngle: " << prop._dFirstAngle.get_deg() << endl;
      cout << "ScanTime: " << prop._dScanTime << endl;
      cout << "TimeIncrement: " << scan->scan_time << endl;
      cout << "MaxLen: " << prop._dMaxRange << endl;

    }
    ++_nReceived;
  }
}

bool CROSGrabber::GetTransform(std_msgs::Header header)
{
  tf::StampedTransform tf_map_baselink,tf_baselink_laser;
  try
  {
    if (_bWaitForTransform) {
      listener_.waitForTransform(map_frame_,base_frame_,header.stamp,ros::Duration(0.1));
      listener_.waitForTransform(base_frame_,header.frame_id,header.stamp,ros::Duration(0.1));
      listener_.lookupTransform(map_frame_,base_frame_,header.stamp,tf_map_baselink);
      listener_.lookupTransform(base_frame_,header.frame_id,header.stamp,tf_baselink_laser);
    }
    else {
      listener_.lookupTransform (map_frame_,base_frame_,header.stamp,tf_map_baselink);
      listener_.lookupTransform (base_frame_, header.frame_id,header.stamp,tf_baselink_laser);
    }
  }
  catch(tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    return false;
  }
  double base_laser_theta=0.0;
  {
    double qz=tf_baselink_laser.getRotation().z();
    double qw=tf_baselink_laser.getRotation().w();
    base_laser_theta=2*atan2(qz,qw);
  }
  tf::Vector3 pos = tf_map_baselink.getOrigin();
  double qz=tf_map_baselink.getRotation().z();
  double qw=tf_map_baselink.getRotation().w();
  double theta=2*atan2(qz,qw);  
  double sec = (double)(header.stamp.nsec*1e-9)+(double)(header.stamp.sec);
  double x = pos.x() * 1000;
  double y = pos.y() * 1000;
  double z = pos.z() * 1000;
  double rad=theta + base_laser_theta;
  rad = (rad>=0) ? fmod(rad,2*M_PI) : -fmod(-rad,2*M_PI);
  if(rad>M_PI) rad -=2*M_PI;
  else if(rad<-M_PI) rad +=2*M_PI;
  double dTheta  = rad;
  _dTempTime = sec;
  _pTempCoords = CCascadedCoords::MakeCasCoords(x, y, z, dTheta, 0, 0, header.frame_id);

  return true;
}

