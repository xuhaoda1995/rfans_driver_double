 /* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <stdio.h>
#include <time.h>
#include <iostream>
#include <string>
#include "rfans_driver/RfansCommand.h"
#include "bufferDecode.h"

#include <pcl_conversions/pcl_conversions.h>


static const int RFANS_POINT_CLOUD_NUM = 1024 ;

static std::vector<SCDRFANS_BLOCK_S> outBlocks ;
static std::vector<RFANS_XYZ_S> outXyzBlocks ;
static sensor_msgs::PointCloud2 outCloud ;

static ros::Publisher  s_output;
static ros::Subscriber s_sub ;

 pcl::PointCloud<pcl::PointXYZI> cloud;


 static DEVICE_TYPE_E s_deviceType = DEVICE_TYPE_NONE;


static void RFansPacketReceived(rfans_driver::RfansPacket pkt) {
  int rtn = 0 ;
  rtn =  SSBufferDec::Depacket(pkt, outCloud,s_output, s_deviceType) ;
  return ;
}


int main ( int argc , char ** argv )
{
  // Initialize the ROS system

  ros::init ( argc , argv , "calculation_node") ;
  ros::NodeHandle nh ;
  SSBufferDec::InitPointcloud2(outCloud) ;

  //node name
  std::string node_name = ros::this_node::getName();

  //advertise name
  std::string advertise_name = "rfans_points";
  std::string advertise_path = node_name + "/advertise_name";
  ros::param::get(advertise_path,advertise_name);
  advertise_path = "rfans_driver/" + advertise_name;
  //ROS_INFO("%s : advertise name %s : %s",node_name.c_str(), advertise_name.c_str(), advertise_path.c_str() );

  //subscribe name
  std::string subscribe_name = "rfans_packets";
  std::string subscribe_path = node_name + "/subscribe_name";
  ros::param::get(subscribe_path, subscribe_name);
  subscribe_path = "rfans_driver/" + subscribe_name;
  ROS_INFO("%s : subscribe name %s : %s",node_name.c_str(), subscribe_name.c_str(), subscribe_path.c_str() );

  //angle durantion
  float angle_duration = 360;
  std::string angle_duration_path = node_name + "/angle_duration";
  ros::param::get(angle_duration_path, angle_duration);
  SSBufferDec::SetAngleDuration(angle_duration);
  ROS_INFO("%s : angle_duration : %f",node_name.c_str(), angle_duration);

  // device type
    std::string device_type_key = node_name + "/device_type";
    std::string device_type_value = "rfans";//default rfans
    ros::param::get(device_type_key, device_type_value);
    ROS_INFO("device_type_key: %s", device_type_key.c_str());
    ROS_INFO("device_type_value: %s", device_type_value.c_str());
    const char *device_type_cfans = "cfans";
    const char *device_type_rfans = "rfans";
    if (0 == strcmp(device_type_value.c_str(), device_type_rfans)) {
      s_deviceType = DEVICE_TYPE_RFANS;
    } else if (0 == strcmp(device_type_value.c_str(), device_type_cfans)) {
      ROS_INFO("device type: cfans");
      s_deviceType = DEVICE_TYPE_CFANS;

      std::string revise_angle_key = node_name + "/revise_angle";
      std::string revise_angle_value = "0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,45,-15,45,-15,0,0,";//default
      ros::param::get(revise_angle_key, revise_angle_value);

      SSBufferDec::initCFansPara(revise_angle_value);
    }

    // save xyz
    std::string save_xyz_key = node_name + "/save_xyz";
    std::string save_xyz_value = "no";//default not save
    ros::param::get(save_xyz_key, save_xyz_value);
    if (0 == strcmp(save_xyz_value.c_str(), "yes")) {
        SSBufferDec::setSaveXYZ(true);
    } else {
        SSBufferDec::setSaveXYZ(false);
    }


  s_sub= nh.subscribe (subscribe_path , RFANS_POINT_CLOUD_NUM, &RFansPacketReceived ) ;
  s_output = nh.advertise<sensor_msgs::PointCloud2>(advertise_path, RFANS_POINT_CLOUD_NUM);
  ros::spin () ;

  return  0;
}
