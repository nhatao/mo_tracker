#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"

#include <iostream>
#include <fstream>
using namespace std;

class CloudToScan
{
public:
  //Constructor
  CloudToScan(): min_height_(-1.2),
                 max_height_(0),
                 angle_min_(-M_PI),
                 angle_max_(M_PI),
                 angle_increment_(0.25/180*M_PI),
                 scan_time_(1.0/10.0),
                 range_min_(0.9),
                 range_max_(30.0),
                 robot_x_min_(-0.9),
                 robot_x_max_( 0.9),
                 robot_y_min_(-2.5),
                 robot_y_max_( 2.5),
                 output_frame_id_("/velodyne"),
                 cloud_topic_("/velodyne_points"),
                 scan_topic_("/scan")
  {
  };

  ~CloudToScan()
  {
  }
  void run() {
    onInit();
    ros::spin();
  }


private:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


  void onInit()
  {
    ros::NodeHandle private_nh("~");

    private_nh.getParam("min_height", min_height_);
    private_nh.getParam("max_height", max_height_);

    private_nh.getParam("angle_min", angle_min_);
    private_nh.getParam("angle_max", angle_max_);
    private_nh.getParam("angle_increment", angle_increment_);
    private_nh.getParam("scan_time", scan_time_);
    private_nh.getParam("range_min", range_min_);
    private_nh.getParam("range_max", range_max_);

    range_min_sq_ = range_min_ * range_min_;

    private_nh.getParam("output_frame_id", output_frame_id_);
    private_nh.getParam("cloud_topic", cloud_topic_);
    private_nh.getParam("scan_topic", scan_topic_);

    private_nh.getParam("robot_x_min", robot_x_min_);
    private_nh.getParam("robot_x_max", robot_x_max_);
    private_nh.getParam("robot_y_min", robot_y_min_);
    private_nh.getParam("robot_y_max", robot_y_max_);


    sub_ = nh_.subscribe(cloud_topic_, 10, &CloudToScan::callback, this);
    pub_ = nh_.advertise<sensor_msgs::LaserScan>(scan_topic_,10);
  };


  void callback(const PointCloud::ConstPtr& cloud)
  {
    sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
    output->header = cloud->header;
    output->header.frame_id = output_frame_id_; // Set output frame. Point clouds come from "optical" frame, scans come from corresponding mount frame
    output->angle_min = angle_min_;
    output->angle_max = angle_max_;
    output->angle_increment = angle_increment_;
    output->time_increment = 0.0;
    output->scan_time = scan_time_;
    output->range_min = range_min_;
    output->range_max = range_max_;

    uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
//    output->ranges.assign(ranges_size, output->range_max + 1.0);
    output->ranges.resize(ranges_size);
    for (size_t i=0; i<ranges_size; ++i) {
      output->ranges[i] = output->range_max + 1.0;
    }
    /*
    cout << "size: " << ranges_size << endl;
    cout << robot_x_min_ << " " << robot_x_max_ << endl;
    cout << robot_y_min_ << " " << robot_y_max_ << endl;
    */

    int nOKNum = 0;

    for (PointCloud::const_iterator it = cloud->begin(); it != cloud->end(); ++it)
    {
      const float &x = it->x;
      const float &y = it->y;
      const float &z = it->z;

      if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
      {
        continue;
      }

      if (z > max_height_ || z < min_height_)
      {
        continue;
      }
      if (((it->x < robot_x_min_) || (robot_x_max_ < it->x)) ||
          ((it->y < robot_y_min_) || (robot_y_max_ < it->y))) 
      {
        double range_sq = x*x+y*y;
        if (range_sq < range_min_sq_) {
          continue;
        }

        double angle = atan2(y, x);
        if (angle < output->angle_min || angle > output->angle_max)
        {
          continue;
        }
        int index = (angle - output->angle_min) / output->angle_increment;

        if ((index < 0) || (index >= output->ranges.size())) {

          cout << angle << " " << output->angle_min << " " << output->angle_increment << " " << index << endl;
        //cout << "something wrong: " << index << endl;
        continue;
  }
        if (output->ranges[index] * output->ranges[index] > range_sq) {
          ++nOKNum;
          output->ranges[index] = sqrt(range_sq);
        }
      }
      else {
//        cout << "in car: " << it->x << " " << it->y << " " << it->z << endl;
      }       
    }
    pub_.publish(output);
  }

  double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_, range_min_sq_;
  std::string output_frame_id_,cloud_topic_,scan_topic_;

  double robot_x_min_;
  double robot_y_min_;
  double robot_x_max_;
  double robot_y_max_;

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};

class CloudToScanPrius {
public:
    //Constructor
    CloudToScanPrius(): min_height_(-1.6),
                        max_height_(-0.4),
                        angle_min_(-M_PI),
                        angle_max_(M_PI),
                        angle_increment_(0.25/180*M_PI),
                        scan_time_(1.0/10.0),
                        range_min_(0.9),
                        range_max_(20.0),
                        robot_x_min_(-0.9),
                        robot_x_max_( 0.9),
                        robot_y_min_(-2.5),
                        robot_y_max_( 2.5),
                        output_frame_id_("/velodyne"),
                        cloud_topic_("/velodyne_points"),
                        scan_topic_("/scan"),
                        _dListupThr(0.5),_nCntThr(2),_dZMinWidth(0.3)
    {}
  void run() {
    onInit();
    ros::spin();
  }

    void onInit()
    {
        ros::NodeHandle private_nh("~");
        /*

        private_nh.getParam("min_height", min_height_);
        private_nh.getParam("max_height", max_height_);

        private_nh.getParam("angle_min", angle_min_);
        private_nh.getParam("angle_max", angle_max_);
        private_nh.getParam("angle_increment", angle_increment_);
        private_nh.getParam("scan_time", scan_time_);
        private_nh.getParam("range_min", range_min_);
        private_nh.getParam("range_max", range_max_);

        private_nh.getParam("listup_thr", _dListupThr);
        private_nh.getParam("z_min_width", _dZMinWidth);
        private_nh.getParam("cnt_thr", _nCntThr);


        private_nh.getParam("output_frame_id", output_frame_id_);
        private_nh.getParam("cloud_topic", cloud_topic_);
        private_nh.getParam("scan_topic", scan_topic_);

        private_nh.getParam("robot_x_min", robot_x_min_);
        private_nh.getParam("robot_x_max", robot_x_max_);
        private_nh.getParam("robot_y_min", robot_y_min_);
        private_nh.getParam("robot_y_max", robot_y_max_);
        */

        sub_ = nh_.subscribe(cloud_topic_, 10, &CloudToScanPrius::callback, this);
        pub_ = nh_.advertise<sensor_msgs::LaserScan>(scan_topic_,10);
    };

    ~CloudToScanPrius(){}
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    void Convert(PointCloud::ConstPtr pCloud, std::vector<double> &rvdResult) {

        int n1 = 0;
        int n2 = 0;
        int nRingNum = 32;
        int nDegNum = pCloud->size()/nRingNum;
        rvdResult.resize(nDegNum, range_max_+1);
        double range_min_sq_ = range_min_ * range_min_;

        for (int n=0; n<nDegNum; ++n) {
            //譛遏ｭ繧呈ｱゅａ繧・
            int nShortest = -1;
            double dShortestSq = DBL_MAX;
            vector<double> vDists(nRingNum, -range_max_); //驕ｩ蠖薙↓蟆上＆縺ｪ謨ｰ
            for (size_t i=0;i<nRingNum; ++i) {

                size_t nPos = n*nRingNum + i;
                const auto &rPos = (*pCloud)[nPos];
                const float &x = rPos.x;
                const float &y = rPos.y;
                const float &z = rPos.z;
                if ( std::isnan(x) || std::isnan(y) || std::isnan(z) ) {
                    continue;
                }
                if (z > max_height_ || z < min_height_) {
                    continue;
                }
                if (((x < robot_x_min_) || (robot_x_max_ < x)) ||
                    ((y < robot_y_min_) || (robot_y_max_ < y))) {                   
                  double range_sq = x*x+y*y;
                  if (range_sq < range_min_sq_) {
                    continue;
                  }
                  
                  if (range_sq < dShortestSq) {
                    nShortest = i;
                    dShortestSq = range_sq;
                  }
                  vDists[i] = sqrt(range_sq);
                  
                  /*
                    double angle = atan2(y, x);
                    if (angle < angle_min_ || angle > angle_max_) {
                    continue;
                    }
                    int index = (angle - angle_min_) / angle_increment_;
                    if (rvdResullt[index] * rvdResult[index] > range_sq)
                    rvdResult[index] = sqrt(range_sq);
                    }
                  */
                }
            }

            if (nShortest >= 0) {
                 
                size_t nPos = n*nRingNum + nShortest;
                const auto &rPos = (*pCloud)[nPos];
                const float &x = rPos.x;
                const float &y = rPos.y;
                const float &z = rPos.z;
                double angle = atan2(y, x);
                if (angle < angle_min_ || angle > angle_max_) {
                    continue;
                }
                double dMinDist = vDists[nShortest];


                //霍晞屬蟾ｮ縺碁明蛟､莉･荳九・繧ゅ・繧偵Μ繧ｹ繝医い繝・・, 蛟区焚縺ｨz縺ｮmin/max繧呈ｱゅａ繧・
                double dZMin = z;
                double dZMax = z;
                int nCnt = 1;

                for (size_t i=0;i<vDists.size(); ++i) {
                    if (i!=nShortest) {
                        if (abs(dMinDist-vDists[i]) < _dListupThr) {
                            size_t nPos = n*nRingNum + i;
                            const auto &rPos = (*pCloud)[nPos];
                            ++nCnt;
                            dZMin = min(dZMin, (double)rPos.z);
                            dZMax = max(dZMax, (double)rPos.z);
                        }
                    }
                }
                if (_dZMinWidth < (dZMax-dZMin)) {
                  //if (true) {
                    int index = (angle - angle_min_) / angle_increment_;
                    rvdResult[index] = dMinDist;
                    ++n1;
                }
                else {
                    ++n2;
                }
            }
        }
        cout << "cnt: " << n1 << " " << n2 << endl;
    }

    void callback(const PointCloud::ConstPtr& cloud)
    {
        sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
        output->header = cloud->header;
        output->header.frame_id = output_frame_id_; // Set output frame. Point clouds come from "optical" frame, scans come from corresponding mount frame
        output->angle_min = angle_min_;
        output->angle_max = angle_max_;
        output->angle_increment = angle_increment_;
        output->time_increment = 0.0;
        output->scan_time = scan_time_;
        output->range_min = range_min_;
        output->range_max = range_max_;

        uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
        vector<double> vdResult;
        Convert(cloud, vdResult);
        for (size_t i=0; i<vdResult.size(); ++i) {
          output->ranges.push_back(vdResult[i]);
        }
        pub_.publish(output);
    }

  double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_;
  std::string output_frame_id_,cloud_topic_,scan_topic_;
  
  double robot_x_min_;
  double robot_y_min_;
  double robot_x_max_;
  double robot_y_max_;

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
    double _dListupThr;
    double _dZMinWidth;
    int _nCntThr;


};


int main(int argc,char **argv)
{
  try {
    ros::init(argc,argv,"cloud_to_scan");
    //CloudToScanPrius cs;
    CloudToScan cs;
    cs.run();
    return 0;
  }
  catch (std::exception &e) {
    cout << "Error: " << e.what() << endl;
  }
}
