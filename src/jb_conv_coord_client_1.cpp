#include "ros/ros.h"
#include "detecting_hand/Conv2DTo3D.h"
#include <cstdlib>
#include <detecting_hand/bounding_box_calculated_center.h>
#include <detecting_hand/msg2dto3d.h>
#include <detecting_hand/msg2dto3d_object.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <iostream>
#include <fstream>



int main(int argc, char **argv){
  ros::init(argc, argv, "conv_coord_client");
  if(argc != 3){
    ROS_INFO("usage: conv_coord X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<detecting_hand::Conv2DTo3D>("conv_coord");
  detecting_hand::Conv2DTo3D srv;
  srv.request.x = atoll(argv[1]);
  srv.request.y = atoll(argv[2]);
  if(client.call(srv)){
    ROS_INFO("NewX: %f NewY: %f NewZ: %f", (float)srv.response.newX, (float)srv.response.newY, (float)srv.response.newZ);
  }
  else{
    ROS_ERROR("Failed to call service conv_coord");
    return 1;
  }
  return 0;
}
