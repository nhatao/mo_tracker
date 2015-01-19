#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>

using namespace std;

#define MO_TRACKER 1

#if MO_TRACKER
#include <mo_tracker/Obstacles.h>
typedef mo_tracker::Obstacles Boxes;
typedef geometry_msgs::Vector3 BoxPointType;
#else
#include <velodyne_tool/ObstacleBoxArray.h>/////////////////////////////////////////////////////////////////////////////////////////
typedef velodyne_tool::ObstacleBoxArray Boxes;
typedef geometry_msgs::Point BoxPointType;
#endif

void InitCommonMarker(visualization_msgs::Marker *marker_p,int ID,double lifetime=0.0)
{
  marker_p->type = marker_p->id = ID;
  marker_p->action = visualization_msgs::Marker::ADD;
  marker_p->lifetime = ros::Duration(lifetime);
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
void ComputeRainbowColor(unsigned int i,unsigned int max_i,double *color)
{
  double x=(double)max_i/3.0;
  if(fabs(x)<1e-6)
  {
    color[0]=1.0;
    color[1]=0.0;
    color[2]=0.0;
    return;
  }
  if(i<(unsigned int)x)
  {
    color[0]=1.0;
    color[1]=(double)i/x;
    color[2]=0.0;
  }
  else if(i<(unsigned int)x*2)
  {
    color[0]=2.0-(double)i/x;
    color[1]=1.0;
    color[2]=-1.0+(double)i/x;
  }
  else
  {
    color[0]=0.0;
    color[1]=3.0-(double)i/x;
    color[2]=1.0;
  }
}

class OBSTACLE_BOX_ARRAY_VISUALIZER
{
private:
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;
  ros::Subscriber box_array_sub_;

  visualization_msgs::Marker marker_box_temp_;
  int max_box_num_;
  void vBoxArrayCB(const Boxes::ConstPtr &box_array)
  {
    visualization_msgs::Marker marker_box=marker_box_temp_;
    marker_box.header=box_array->header;
#if MO_TRACKER
        marker_box.points.resize(box_array->obstacle.size()*8);
        marker_box.colors.resize(box_array->obstacle.size()*8);
        unsigned int size=box_array->obstacle.size();
#else
        marker_box.points.resize(box_array->box.size()*8);
        marker_box.colors.resize(box_array->box.size()*8);
        unsigned int size=box_array->box.size();
#endif
    if(max_box_num_>0)
    {
      if(size>max_box_num_)size=max_box_num_;
    }
    for(unsigned int i=0;i<size;i++)
    {
      double color[3];
      ComputeRainbowColor(i,size-1,color);
      //a,r,g,b
      for(unsigned int j=0;j<8;j++)SetColor(&marker_box.colors[8*i+j], color[0], color[1], color[2], 1.0);
      for(unsigned int j=0;j<4;j++)
      {
        unsigned int j2=(j<3)?(j+1):(0);
#if MO_TRACKER
                marker_box.points[8*i+2*j].x=box_array->obstacle[i].position[j].x;
                marker_box.points[8*i+2*j].y=box_array->obstacle[i].position[j].y;
                marker_box.points[8*i+2*j].z=0.0;
                marker_box.points[8*i+2*j+1].x=box_array->obstacle[i].position[j2].x;
                marker_box.points[8*i+2*j+1].y=box_array->obstacle[i].position[j2].y;
                marker_box.points[8*i+2*j+1].z=0.0;
#else
                marker_box.points[8*i+2*j].x=box_array->box[i].position[j].x;
                marker_box.points[8*i+2*j].y=box_array->box[i].position[j].y;
                marker_box.points[8*i+2*j].z=0.0;
                marker_box.points[8*i+2*j+1].x=box_array->box[i].position[j2].x;
                marker_box.points[8*i+2*j+1].y=box_array->box[i].position[j2].y;
                marker_box.points[8*i+2*j+1].z=0.0;

 #endif

                }
    }
    marker_pub_.publish(marker_box);
  }
public:
  OBSTACLE_BOX_ARRAY_VISUALIZER()
  {
    std::string marker_topic="/box_marker";
    std::string box_topic="/static_obstacle";
    max_box_num_=20;
    ros::NodeHandle private_nh("~");
    private_nh.param("marker_topic",marker_topic,marker_topic);
    private_nh.param("obstacle_box_topic",box_topic,box_topic);
    private_nh.param("max_box_num",max_box_num_,max_box_num_);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic,1);
    box_array_sub_ = nh_.subscribe(box_topic,1,&OBSTACLE_BOX_ARRAY_VISUALIZER::vBoxArrayCB,this);
    InitCommonMarker(&marker_box_temp_,visualization_msgs::Marker::LINE_LIST,0.15);
    SetScale(&marker_box_temp_, 0.2, 0.0, 0.0);
    marker_box_temp_.ns="box";
    printf("visualize start\n");

    cout << "marker_topic = "  << marker_topic << endl;
    cout << "obstacle_box_topic = " << box_topic << endl;
    
  }
  ~OBSTACLE_BOX_ARRAY_VISUALIZER()
  {
  }
};

int main(int argc,char **argv)
{
  ros::init(argc,argv,"obstacle_box_array_visualizer");
  OBSTACLE_BOX_ARRAY_VISUALIZER obav;
  ros::spin();
  return (0);
}
