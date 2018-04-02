
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <nmea_msgs/Sentence.h>
#include <visualization_msgs/Marker.h>
#include "velodyne_pointcloud/point_types.h"
#include "velodyne_pointcloud/rawdata.h"
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

static std::string PARENT_FRAME;
static std::string CHILD_FRAME;
static std::string POINTS_TOPIC;
static int SCAN_NUM;
static std::string OUTPUT_DIR;

static pcl::PointCloud<velodyne_pointcloud::PointXYZIR> map;
static tf::TransformListener *tf_listener;
static std::string filename;

static int added_scan_num = 0;
static int map_id = 0;
static int count = 0;

static ros::Publisher points_transformed_pub;
static ros::Publisher marker_pub;

std::vector<std::string> split(const std::string &string)
{
  std::vector<std::string> str_vec_ptr;
  std::string token;
  std::stringstream ss(string);

  while (getline(ss, token, ','))
    str_vec_ptr.push_back(token);

  return str_vec_ptr;
}

void nmea_callback(const nmea_msgs::Sentence::ConstPtr &input)
{
  std::vector<std::string> nmea = split(input->sentence);
  std::string tag = nmea.at(0);

  if(tag == "$PASHR") {
    double heading = stod(nmea.at(2));
    double roll = stod(nmea.at(4));
    double pitch = stod(nmea.at(5));
    std::cout << "heading: " << heading << " , roll: " << roll << " , pitch: " << pitch << std::endl;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = input->header.stamp;
    marker.id = 0;

    marker.type = visualization_msgs::Marker::ARROW;
    marker.pose.position.x = 3160.0;
    marker.pose.position.y = -75475.0;
    marker.pose.position.z = 210.0;
    tf::Quaternion q;
    q.setRPY(M_PI*roll/180.0, M_PI*pitch/180.0, M_PI*(90.0-heading)/180.0);
    quaternionTFToMsg(q, marker.pose.orientation);

    marker.scale.x = 3.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker_pub.publish(marker);

  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nmea2tf");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  /*
  private_nh.getParam("parent_frame", PARENT_FRAME);
  private_nh.getParam("child_frame", CHILD_FRAME);
  private_nh.getParam("points_topic", POINTS_TOPIC);
  private_nh.getParam("scan_num", SCAN_NUM);
  private_nh.getParam("output_dir", OUTPUT_DIR);

  std::cout << "parent_frame: " << PARENT_FRAME << std::endl;
  std::cout << "child_frame: " << CHILD_FRAME << std::endl;
  std::cout << "points_topic: " << POINTS_TOPIC << std::endl;
  std::cout << "scan_num: " << SCAN_NUM << std::endl;
  std::cout << "output_dir: " << OUTPUT_DIR << std::endl;
*/
  tf_listener = new tf::TransformListener();

  marker_pub = nh.advertise<visualization_msgs::Marker>("/marker", 1000);
  ros::Subscriber nmea_sub = nh.subscribe("/nmea_sentence", 10, nmea_callback);

  ros::spin();

  return 0;
}
