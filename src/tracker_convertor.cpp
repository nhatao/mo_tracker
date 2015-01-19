#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <mo_tracker/Trackers.h>
#include <sstream>
#include <boost/unordered_map.hpp>
const std::string MARKER_TOPIC="/tracker_marker";
const std::string TRACKER_TOPIC="/human_track";
//typedef std::map<unsigned int,visualization_msgs::Marker> MapMarker;
typedef boost::unordered_map<unsigned int,visualization_msgs::Marker> MapMarker;

class TRACKER_CONVERTOR
{
private:
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;
  ros::Subscriber tracker_sub_;

  std::string marker_topic_;
  std::string tracker_topic_;

  std_msgs::ColorRGBA color_[2];
  visualization_msgs::Marker marker_arrow_;
  visualization_msgs::Marker marker_sphere_;
  visualization_msgs::Marker marker_trj_tmp_;
  MapMarker marker_trj_;

  void InitCommonMarker(visualization_msgs::Marker *marker_p,int ID)
  {
    marker_p->type = marker_p->id = ID;
    marker_p->action = visualization_msgs::Marker::ADD;
    marker_p->lifetime = ros::Duration(0.5);
  }
  void SetScale(visualization_msgs::Marker *marker_p,double x,double y,double z)
  {
    marker_p->scale.x = x;
    marker_p->scale.y = y;
    marker_p->scale.z = z;
  }
  void SetColor(std_msgs::ColorRGBA *color,double r,double g,double b,double a)
  {
    color->r=r;
    color->g=g;
    color->b=b;
    color->a=a;
  }
  void InitSphereList(visualization_msgs::Marker *marker_p)
  {
    InitCommonMarker(marker_p,visualization_msgs::Marker::SPHERE_LIST);
    marker_p->ns = "position";
    SetScale(marker_p, 0.2, 0.2, 0.2);
    marker_p->color = color_[0];
  }
  void InitLineList(visualization_msgs::Marker *marker_p)
  {
    InitCommonMarker(marker_p,visualization_msgs::Marker::LINE_LIST);
    marker_p->ns = "velosity";
    SetScale(marker_p, 0.1, 0.0, 0.0);
    marker_p->color = color_[0];
  }
  void InitLineStrip(visualization_msgs::Marker *marker_p)
  {
    InitCommonMarker(marker_p,visualization_msgs::Marker::LINE_STRIP);
    marker_p->ns = "trajectory";
    SetScale(marker_p, 0.02, 0.0, 0.0);
    marker_p->color = color_[1];
  }

  void KillInvalidLineStrip(const mo_tracker::Trackers::ConstPtr &tracker)
  {
    if(marker_trj_.empty())return;
    std::set<int> trk_id;
    for(unsigned int i=0;i<tracker->tracker.size();i++)
      trk_id.insert(tracker->tracker[i].id);
    std::vector<int> kill_id;
    for(MapMarker::iterator it=marker_trj_.begin();it!=marker_trj_.end();it++)
    {
      if(trk_id.find(it->first)==trk_id.end())
        kill_id.push_back(it->first);
    }
    for(unsigned int i=0;i<kill_id.size();i++)
    {
      marker_trj_.erase(kill_id[i]);
    }
  }
  void SetLineStrip(const unsigned int ID,std_msgs::Header header,geometry_msgs::Point p)
  {
    bool empty=marker_trj_.empty();
    int old_id=-1;
    MapMarker::iterator it;
    if(!empty)
    {
      it=marker_trj_.find(ID);
      if(it==marker_trj_.end())old_id=1;
      else old_id=0;
    }

    if(empty||old_id==1)
    {
      if(marker_trj_tmp_.points.empty())marker_trj_tmp_.points.push_back(p);
      else marker_trj_tmp_.points[0]=p;
      marker_trj_tmp_.header=header;
      marker_trj_.insert(MapMarker::value_type(ID,marker_trj_tmp_));
    }
    else if(old_id==0)
    {
      it->second.header=header;
      it->second.points.push_back(p);
      if(it->second.points.size()>200)it->second.points.erase(it->second.points.begin());
    }
  }

  void TrackerCallback(const mo_tracker::Trackers::ConstPtr &tracker)
  {
    KillInvalidLineStrip(tracker);
    marker_sphere_.points.resize(tracker->tracker.size());
    marker_arrow_.points.resize(2*tracker->tracker.size());
    for(unsigned int i=0;i<tracker->tracker.size();i++)
      {
  geometry_msgs::Point p1;
  p1.x=tracker->tracker[i].position.x;
  p1.y=tracker->tracker[i].position.y;
  p1.z=tracker->tracker[i].position.z;
  marker_sphere_.points[i]=p1;
  geometry_msgs::Point p2(p1);
  p2.x+=tracker->tracker[i].velocity.x;
  p2.y+=tracker->tracker[i].velocity.y;
  marker_arrow_.points[2*i]=p1;
  marker_arrow_.points[2*i+1]=p2;
        marker_sphere_.header=marker_arrow_.header=tracker->header;
        SetLineStrip(tracker->tracker[i].id,tracker->header,p1);
      }
    
    visualization_msgs::MarkerArray msg;
    msg.markers.resize(marker_trj_.size()+2);
    MapMarker::iterator it=marker_trj_.begin();
    msg.markers[0]=marker_sphere_;
    msg.markers[1]=marker_arrow_;
    for(int i=0;i<marker_trj_.size();i++,it++)
    {
      std::ostringstream oss;
      oss<<"/#"<<i;
      msg.markers[2+i]=it->second;
      msg.markers[2+i].ns+=oss.str();
    }
    marker_pub_.publish(msg);
  }

public:
  TRACKER_CONVERTOR()
  {
    ros::NodeHandle n_private_("~");
    n_private_.param("marker_topic",marker_topic_,MARKER_TOPIC);
    n_private_.param("tracker_topic",tracker_topic_,TRACKER_TOPIC);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_,1);
    tracker_sub_ = nh_.subscribe(tracker_topic_,1,&TRACKER_CONVERTOR::TrackerCallback,this);

    //setting color(r,g,b,a)
    SetColor(&color_[0],0.0,0.0,1.0,1.0);
    SetColor(&color_[1],1.0,1.0,1.0,1.0);
    InitSphereList(&marker_sphere_);
    InitLineList(&marker_arrow_);
    InitLineStrip(&marker_trj_tmp_);
    printf("start convert...\n");
  }
  ~TRACKER_CONVERTOR()
  {
    ros::spin();
    printf("\nend\n");
  }
};

int main(int argc,char **argv)
{
  ros::init(argc,argv,"tracker_convertor");
  TRACKER_CONVERTOR tc;
  return (0);
}
