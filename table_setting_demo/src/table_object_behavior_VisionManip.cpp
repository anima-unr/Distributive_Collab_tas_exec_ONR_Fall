/*
baxter_demos
Copyright (C) 2015  Luke Fraser

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <cmath>
#include "table_setting_demo/log.h"
#include "table_setting_demo/table_object_behavior_VisionManip.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "visualization_msgs/Marker.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/JointState.h"
#include "table_setting_demo/table_setting_demo_types.h"
#include "table_setting_demo/pick_and_place.h"
#include "table_setting_demo/object_request.h"
#include "table_setting_demo/pick_and_place_state.h"
#include "table_setting_demo/object_position.h"
#include "table_setting_demo/pick_and_place_stop.h"
#include "table_setting_demo/ObjectTransformation.h"

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


namespace pr2 {
typedef enum STATE {
  APPROACHING = 0,
  PICKING,
  PICKED,
  PLACING,
  PLACED,
  NEUTRAL,
  IDLE
} STATE_t;
}  // namespace pr2

namespace task_net {

static const char *dynamic_object_str[] = {
  // "cup",
  // "bowl",
  // "soda",
  // "fork",
  // "spoon",
  // "knife"
};
static const char *static_object_str[] = {
  // // "fork",
  // // "spoon",
  // // "knife",
  // // //"cup",
  // // "bowl",
  // // "soda",
  // // "neutral",
  // // "placemat",
  // // "wineglass",
  // // "plate",
  // // // tkdjflkajsd;fnaskdf;
  // // "Cup",
  // // "Tea",
  // // "Sugar",
  // // "Left_Bread",
  // // "Right_Bread",
  // // "Meat",
  // // "Lettuce"
  // // DARS DEMO
  // "teddy_bear",
  // // "orange",
  // "sports_ball",
  // "clock",
  // // "bottle",
  // "scissors",
  // "cup",
  // // "bowl",
  "Cup",
  "Tea_Pot",
  //"Sugar",
  "Burger",
  //"Sandwich",
  "Apple",
  //"Orange"
};

TableObject_VisionManip::TableObject_VisionManip() : arm_group_{"right_arm"} {  ROS_ERROR("START OF TableObject_VisionManip CONSTRUCTOR");}
TableObject_VisionManip::TableObject_VisionManip(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    std::string object,
    std::string mutex_topic,
    std::vector<float> pos,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : 
object_pos(pos), object_(object),
Behavior(name,
      peers,
      children,
      parent,
      state, 
      object),
       mut(name.topic.c_str(), mutex_topic), nh_(), tf_listener_(),
       arm_group_{"right_arm"} {

  // flag saying whether the ROS publishers/listeners have been created

  ROS_ERROR("START OF TableObject_VisionManip CONSTRUCTOR");

  ready_to_publish_ = false;

  object_ = object;
  // object_pos = pos;
  geometry_msgs::PoseStamped currentPose;

  ROS_ERROR("2 START OF TableObject_VisionManip CONSTRUCTOR");
  neutral_object_pos = {0.0, 0.0, 0.0};
  currentPose = arm_group_.getCurrentPose();
  neutral_object_pos[0] = currentPose.pose.position.x; //TODO: Don't want to use neutral pos, need it to be current pose of robot!!! - set a moveit thing in class and then can call from all these things to get current pose of robot to set to "neutral??"
  neutral_object_pos[1] = currentPose.pose.position.y; //TODO: Don't want to use neutral pos, need it to be current pose of robot!!! - set a moveit thing in class and then can call from all these things to get current pose of robot to set to "neutral??"
  neutral_object_pos[2] = currentPose.pose.position.z; //TODO: Don't want to use neutral pos, need it to be current pose of robot!!! - set a moveit thing in class and then can call from all these things to get current pose of robot to set to "neutral??"
  // neutral_object_pos = neutral_pos; //TODO: Don't want to use neutral pos, need it to be current pose of robot!!! - set a moveit thing in class and then can call from all these things to get current pose of robot to set to "neutral??"
  object_id_ = "";

  // Check size of initial position of objects
  if (pos.size() <= 0)
    object_pos = std::vector<float>(3);

  ROS_ERROR("3 START OF TableObject_VisionManip CONSTRUCTOR");

  // set objects to static
  std::vector<std::string> static_objects_ = std::vector<std::string>(
    static_object_str,
    static_object_str + sizeof(static_object_str) / sizeof(char*));
  dynamic_object = true;
  for (int i = 0; i < static_objects_.size(); ++i) {
    if (object == static_objects_[i]) {
      dynamic_object = false;
      break;
    }
  }
  // set object_id
  if (!dynamic_object) {
    object_id_ = object;
  }

  // set root/manip frames
  nh_.getParam("root_frame", root_frame_);
  nh_.getParam("manip_frame", manip_frame_);
  //tf_listener_ = new TransformListener();

  // debugging - declare publisher for manip markers
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/markers",1000);
  sleep(3);
  ready_to_publish_ = true;
  ROS_ERROR("END OF TableObject_VisionManip CONSTRUCTOR");

}
TableObject_VisionManip::~TableObject_VisionManip() {}

void TableObject_VisionManip::UpdateActivationPotential() {
  float dist;
  // ROS_WARN("TableObject::UpdateActivationPotential was called!!!\n");

  
  // manipulator position defaults to neutral_obj_pos
  float mx, my, mz, ox, oy, oz;
  if( neutral_object_pos.size() == 0 ) {
    ROS_WARN( "neutral_object_pos size is 0, that's weird..." );
    mx = my = mz = 0;
    state_.activation_potential = 0.00001;
    return;
  }
  else {
    if( neutral_object_pos.size() == 3 ) {
      mx = neutral_object_pos[0];
      my = neutral_object_pos[1];
      mz = neutral_object_pos[2];
    }
    else{
      ROS_ERROR( "neutral_object_pos size is != 3...setting activation to 0" );
      state_.activation_potential = 0.00001;
      return;
    }
  }

  if( object_pos.size() == 0 ) {
    ROS_WARN( "object_pos size is 0, that's weird..." );
    ox = oy = oz = 0;
    state_.activation_potential = 0.00001;
    return;
  }
  else {
    if( object_pos.size() == 3 ) {
      ox = object_pos[0];
      oy = object_pos[1];
      oz = object_pos[2];
    }
    else{
      ROS_ERROR( "object_pos size is != 3...setting activation to 0" );
      state_.activation_potential = 0.00001;
      return;
    }
}

  // // LUKES WAY TO GET ARM POSITION.......
  // // get PR2 hand position (and store in mx, my, mz)
  // tf::StampedTransform transform;
  // try{
  //   //ROS_INFO( "trying transform" );
  //   tf_listener_.lookupTransform(root_frame_, manip_frame_, ros::Time(0), transform);
  //   mx = transform.getOrigin().x();
  //   my = transform.getOrigin().y();
  //   mz = transform.getOrigin().z();
  //   //ROS_INFO( "got transformation: %0.2f %0.2f %0.2f", mx, my, mz);
  // }
  // catch( tf::TransformException ex)
  // {
  //   ROS_WARN( "could not get transform between [%s] and [%s] (%s), relying on neutral_obj_pos", root_frame_.c_str(), manip_frame_.c_str(), ex.what());
  // }

  // JB Way of getting arm position through moveit!!!
  geometry_msgs::PoseStamped currentPose;
  currentPose = arm_group_.getCurrentPose();
  mx = currentPose.pose.position.x; //TODO: Don't want to use neutral pos, need it to be current pose of robot!!! - set a moveit thing in class and then can call from all these things to get current pose of robot to set to "neutral??"
  my = currentPose.pose.position.y; //TODO: Don't want to use neutral pos, need it to be current pose of robot!!! - set a moveit thing in class and then can call from all these things to get current pose of robot to set to "neutral??"
  mz = currentPose.pose.position.z; //TODO: Don't want to use neutral pos, need it to be current pose of robot!!! - set a moveit thing in class and then can call from all these things to get current pose of robot to set to "neutral??"

  // debugging: publish TF frames to make sure objects positions are understood
  if( ready_to_publish_ )
  {
    visualization_msgs::Marker marker;

    // marker to show object location
    marker.header.frame_id = root_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "manip_shapes";
    marker.id = mask_.type * 1000 + mask_.robot * 100 + mask_.node * 10 + 0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position.x = ox;
    marker.pose.position.y = oy;
    marker.pose.position.z = oz;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    marker_pub_.publish(marker);

    // marker to show hand location
    marker.id = mask_.type * 1000 + mask_.robot * 100 + mask_.node * 10 + 1;
    marker.pose.position.x = mx;
    marker.pose.position.y = my;
    marker.pose.position.z = mz;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker_pub_.publish(marker);

  }

  double c1 = 1.0; // weight for distance
  double c2 = 0.001; //weight for suitability
  dist = hypot(my - oy, mx - ox);

  if( fabs(dist) > 0.00001 )
    state_.activation_potential = ( c1 * (1.0f / dist)) + (c2 * state_.suitability); // WITH SUITABILITY
    //state_.activation_potential = ( c1 * (1.0f / dist));     //WIHTOUT IT
  else state_.activation_potential = 0.00001;

  // ROS_INFO("OBJ(%s): updating activation potential: %0.2f", object_.c_str(), state_.activation_potential);

  // ROS_INFO("object_pos: %f %f %f", object_pos[0],object_pos[1],object_pos[2]);
  // ROS_INFO("object_pos: %f %f %f", neutral_object_pos[0],neutral_object_pos[1],neutral_object_pos[2]);
  // ROS_INFO("x %f y %f z %f dist %f activation_potential %f", x, y, z, dist, state_.activation_potential);
}

void TableObject_VisionManip::PickAndPlace(std::string object) {
  table_setting_demo::pick_and_place msg;
  msg.request.object = object;
  if (ros::service::call("pick_and_place_object", msg)) {

  }
}
bool TableObject_VisionManip::Precondition() {
  // check if dynamic object
  if (dynamic_object) {
    table_setting_demo::object_request msg;
    table_setting_demo::object_position pos_msg;
    pos_msg.request.object_id = object_;
    LOG_INFO("Dynamic Object Checking Precondition, [%s]", object_.c_str());
    // Check if object available in scene
    if (ros::service::call("qr_get_object_position", pos_msg)) {
      if (pos_msg.response.position.size() > 0) {
        object_pos[0] = pos_msg.response.position[0] + pos_msg.response.position[2] / 2.0;
        object_pos[1] = pos_msg.response.position[1] + pos_msg.response.position[3] / 2.0;
        object_pos[2] = 0;
      }
    } else {
      LOG_INFO("SERVICE: [%s] - Not responding!", "qr_get_object_position");
    }
  }

  // if( object_ == "cup") {
  //   sleep(1.0);
  //   return false;
  // }
  return true;
}
bool TableObject_VisionManip::ActivationPrecondition() {
  ros::Duration(2).sleep();
  return mut.Lock(state_.activation_potential);
}

bool TableObject_VisionManip::PickAndPlaceDone() {
  table_setting_demo::pick_and_place msg;
  msg.request.object = object_;
  ros::service::call("pick_and_place_check", msg);
  return msg.response.success;
}

void TableObject_VisionManip::Work() {
  // check that obejct tracker is still good
  if (dynamic_object) {
  table_setting_demo::object_position pos_msg;
    pos_msg.request.object_id = object_;
    bool tracked = false;
    boost::this_thread::sleep(boost::posix_time::millisec(1000));
    while (!tracked) {
      if (ros::service::call("qr_get_object_position", pos_msg)) {
        if (pos_msg.response.position.size() > 0) {
          tracked = true;
        }
      }
    }
    LOG_INFO("Waiting to pick up object [%s]!", object_.c_str());
    boost::this_thread::sleep(boost::posix_time::millisec(6000));
    object_id_ = object_;
  }
  PickAndPlace(object_id_.c_str());
  while (!PickAndPlaceDone()) {
    boost::this_thread::sleep(boost::posix_time::millisec(500));
      // ROS_INFO("TableObject::Work: waiting for pick and place to be done!");
  }
  // Check if succeeded and try again
  mut.Release();
  ROS_INFO("TableObject::Work: Released mutex!\n\n\n\n\n\n\n\n\n\n\n\n");//checking this line get printed 
}
float CalcPositionDistance_VisionManip(std::vector<float> pos_a, std::vector<float> pos_b) {
  // LOG_INFO("TX: %f TY: %f TZ: %f, 3X: %f 3Y: %f 3Z: %f", pos_a);
  float x = pow(pos_a[0] - pos_b[0], 2);
  float y = pow(pos_a[1] - pos_b[1], 2);
  float z = 0;//pow(pos_a[2] - pos_b[2], 2);
  return sqrt(x + y + z);
}

bool TableObject_VisionManip::CheckWork() {
  table_setting_demo::pick_and_place_state msg;
  table_setting_demo::pick_and_place view_msg;
  table_setting_demo::object_position pos_msg;
  float distance_thresh = 15;
  float dist;
  if (dynamic_object) {
    if(ros::service::call("pick_and_place_state", msg)) {
      if (msg.response.state == pr2::APPROACHING) {
        LOG_INFO("Approaching object: %s - Checking Object Availability",
          object_.c_str());
        // Check if the object is still in view
        // TODO: consider self intersection with objects blocking the view.
        // if (ros::service::call("qr_object_inview", view_msg)) {
        LOG_INFO("OBJECT IN VIEW!!!");
          // Check if the object is in the same place with in reason
          pos_msg.request.object_id = object_;
        if (ros::service::call("qr_get_object_position", pos_msg)) {
          std::vector<float> track_pos(3);
          track_pos[0] = pos_msg.response.position[0] + pos_msg.response.position[2] / 2.0;
          track_pos[1] = pos_msg.response.position[1] + pos_msg.response.position[3] / 2.0;
          track_pos[2] = 0;
          dist = CalcPositionDistance_VisionManip(track_pos, object_pos);
          LOG_INFO("DISTANCE MEASURE: [%F]!!!!!!!!!!!!!!!!!!!!!", dist);
          if (dist < distance_thresh) {
            return true;
          } else {
            ROS_DEBUG("RESETING  BEHAVIOR!!!!!!!!!!!!!!");
            return false;
          }
        }
      }
    }
  }
  return true;
}

void TableObject_VisionManip::UndoWork() {
  LOG_INFO("UNDOING WORK!!");
  table_setting_demo::pick_and_place_stop msg;
  ros::service::call("pick_and_place_stop", msg);
  mut.Release();
  // update object position
  if (dynamic_object) {
    table_setting_demo::object_request msg;
    table_setting_demo::object_position pos_msg;
    pos_msg.request.object_id = object_;
    LOG_INFO("Renewing position, [%s]", object_.c_str());
    // Check if object available in scene
    if (ros::service::call("qr_get_object_position", pos_msg)) {
      if (pos_msg.response.position.size() > 0) {
        object_pos[0] = pos_msg.response.position[0] + pos_msg.response.position[2] / 2.0;
        object_pos[1] = pos_msg.response.position[1] + pos_msg.response.position[3] / 2.0;
        object_pos[2] = 0;
      }
    } else {
      LOG_INFO("SERVICE: [%s] - Not responding!", "qr_get_object_position");
    }
  }
}

// Release mutex for undo stuffs
void TableObject_VisionManip::ReleaseMutexLocs() {
 ROS_DEBUG("Releasing mutex from tableobjbehavior");

 ROS_WARN("Releasing mutex from tableobjbehavior\n\n\n\n\n\n\n\n");

 // release the Mutex for the remote mutex
 mut.Release();
 return;
}

}  // namespace task_net
