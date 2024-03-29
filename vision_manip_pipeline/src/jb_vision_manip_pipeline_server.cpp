#include "ros/ros.h"
#include "vision_manip_pipeline/VisionManip.h"
#include "sensor_msgs/PointCloud2.h"
#include <iostream>
#include <algorithm>

#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Float32.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include "vision_manip_pipeline/Conv2DTo3D.h"
// #include "vision_manip_pipeline/GetObjLoc.h"
#include "vision_manip_pipeline/GetGrasp.h"
#include "vision_manip_pipeline/PubWorkspace.h"

#include "active_vision_msgs/Vision_Service.h"
#include "active_vision_msgs/Vision_Message.h"

#include <Eigen/Dense>
#include <signal.h>
#include <tf/LinearMath/Quaternion.h>
#include <Eigen/Core>
#include <stdlib.h>

//**************************************
// TODO_AAMS:
//One potential clean up in the future (once we can upgrade to Kinetic) is to look at this option for integrating GPD with moveit
// through the pickup actions in moveit: https://github.com/TAMS-Group/moveit_gpd_pick_object/blob/master/src/plan_grasps_service.cpp
// or other things discussed here https://github.com/ros-planning/moveit/issues/566
//**************************************

//----------------------------------------------------------------------
// globals
//----------------------------------------------------------------------
// ros::NodeHandle n;
tf::TransformListener *t;

// ros::ServiceClient *conv2DTo3DClient_pntr;
// ros::ServiceClient *objLocClient_pntr;
ros::ServiceClient *pubWorkspaceClient_pntr;
ros::ServiceClient *getGraspClient_pntr;
ros::ServiceClient *pouryaClient_pntr;



//----------------------------------------------------------------------
// helper functions
//----------------------------------------------------------------------


geometry_msgs::PoseStamped getPoseTrans(double x, double y, double z, std::vector<double> ori, const std::string old_frame, const std::string new_frame){

  ros::Time now = ros::Time(0);
  t->waitForTransform(new_frame, old_frame, now, ros::Duration(3.0));

  geometry_msgs::PoseStamped pnt;
  pnt.header.frame_id = old_frame;
  pnt.header.stamp = now;
  pnt.pose.position.x = x;
  pnt.pose.position.y = y;
  pnt.pose.position.z = z;
  pnt.pose.orientation.w = ori[0]; //TODO: double check order of w,x,y,z!
  pnt.pose.orientation.x = ori[1]; //TODO: double check order of w,x,y,z!
  pnt.pose.orientation.y = ori[2]; //TODO: double check order of w,x,y,z!
  pnt.pose.orientation.z = ori[3]; //TODO: double check order of w,x,y,z!

  geometry_msgs::PoseStamped newPnt;
  t->transformPose(new_frame, pnt, newPnt);

  std::cout << "\nTRANSFORM: " << newPnt << '\n';
  return newPnt;
}

// ==========================================================

geometry_msgs::PointStamped transPoint(double x, double y, double z, const std::string old_frame, const std::string new_frame){
  ros::Time now = ros::Time(0);
    // system(" sudo invoke-rc.d chrony restart");
    // sleep(3);"/head_mount_kinect_rgb_optical_frame", "/test"
  t->waitForTransform(new_frame, old_frame, now, ros::Duration(3.0));

  geometry_msgs::PointStamped pnt;
  pnt.header.frame_id = old_frame;
  pnt.header.stamp = now;
  pnt.point.x = x;
  pnt.point.y = y;
  pnt.point.z = z;

  geometry_msgs::PointStamped newPnt;
  t->transformPoint(new_frame, pnt, newPnt);

  std::cout << "\nTRANSFORM: " << newPnt << '\n';
  return newPnt;
}

// ==========================================================

bool moveArm(geometry_msgs::PoseStamped newApp, geometry_msgs::PoseStamped newPnt){

  moveit::planning_interface::MoveGroup group("right_arm");
  printf("Move it TEST0\n");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<moveit_msgs::CollisionObject> collision_objects_;
  sleep(1);
  moveit_msgs::CollisionObject collision_object1;
  collision_object1.header.frame_id = "/base_link";

  // -------
  /* Define a table to add to the world. */
  /* The id of the object is used to identify it. */
  collision_object1.id = "table";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1;
  primitive.dimensions[1] = 2;
  primitive.dimensions[2] = 0.85;

  /* A pose for the table (specified relative to frame_id) */
  geometry_msgs::Pose table_pose;
  table_pose.orientation.w = 0;
  table_pose.position.x =  0.65;
  table_pose.position.y =  0;
  table_pose.position.z = -0.62; //objects_n3_v2.bin

  collision_object1.primitives.push_back(primitive);
  collision_object1.primitive_poses.push_back(table_pose);
  collision_object1.operation = collision_object1.ADD;
  collision_objects_.push_back(collision_object1);

  //----------------------------------------
  // Add objects to world
  // if collision_objects_.primitives.empty()
  ROS_INFO("Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects_);
  // sleep(20);
  ROS_INFO("out of set table function");

  //------------------
  printf("Move to approach\n");

  geometry_msgs::Pose pose_target;
  pose_target.orientation.x = newApp.pose.orientation.x;
  pose_target.orientation.y = newApp.pose.orientation.y;
  pose_target.orientation.z = newApp.pose.orientation.z;
  pose_target.orientation.w = newApp.pose.orientation.w;
  pose_target.position.x = newApp.pose.position.x;
  pose_target.position.y = newApp.pose.position.y;
  pose_target.position.z = newApp.pose.position.z;

  group.setPoseTarget(pose_target);

  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

  if( success) {
    /* Sleep to give Rviz time to visualize the plan. */
    printf("\tPlanning...");
    sleep(1.0);
    // printf("\tMoving...");
    // group.move();
    // sleep(5.0);
  }
  else {
    printf("Moving to next grasp!\n");
    return false;
  }

 //------------------
  printf("Move to pick\n");

  pose_target.orientation.x = newPnt.pose.orientation.x;
  pose_target.orientation.y = newPnt.pose.orientation.y;
  pose_target.orientation.z = newPnt.pose.orientation.z;
  pose_target.orientation.w = newPnt.pose.orientation.w;
  pose_target.position.x = newPnt.pose.position.x;
  pose_target.position.y = newPnt.pose.position.y;
  pose_target.position.z = newPnt.pose.position.z;

  group.setPoseTarget(pose_target);

  success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

  if( success) {
    /* Sleep to give Rviz time to visualize the plan. */
    printf("\tPlanning...");
    sleep(1.0);
    // printf("\tMoving...");
    // group.move();
    // sleep(5.0);
  }
  else {
    printf("Moving to next grasp!\n");
    return false;
  }

  return true;

}

// ==========================================================

bool checkInBounds(geometry_msgs::PointStamped newPnt) {

  // pr2 arm is 100 cm.... 10^2 = x^2 + y^2 .... so if x^2 + y^2 < 0.9 then okay, else return 0?
  // but first need to convert to r_torso_lift_side_plate_link frame?? - then can just do the mathhhh

  // convert to r_torso_lift_side_plate_link frame
  geometry_msgs::PointStamped transPnt = transPoint(newPnt.point.x, newPnt.point.y, newPnt.point.z, "/test", "/right_arm_mount");

  // check if in bounds
  double threshold = 1.0;
  double dist = 1.0;

  dist = (transPnt.point.x * transPnt.point.x) + (transPnt.point.y * transPnt.point.y);

  if( dist < threshold ) {
    return true;
  }
  else {
    return false;
  }

}

// ==========================================================

// std::vector<double> rotationQuat(std::vector<double> approach, std::vector<double> axis,
//   std::vector<double> binormal){
//     double r1, r2;
//     std::vector<double> orientation = {0,0,0,1};
//     Eigen::Vector3d up(binormal[0], binormal[1], binormal[2]);
//     Eigen::Vector3d forward(axis[0], axis[1], axis[2]);

//     Eigen::Vector3d m2 = forward;
//     Eigen::Vector3d m0 = up.cross(forward);
//     Eigen::Vector3d m1 = m2.cross(m0);

//     double r0 = m0[0] + m0[1] + m0[2];
//     if( r0 > 0 )
//     {
//           double r1 = sqrt(r0 + 1.0);
//           orientation[3] = r1 * 0.5;
//           r1 = 0.5 / r1;
//           orientation[0] = (m1[2] - m2[1]) * r1;
//           orientation[1] = (m2[0] - m0[2]) * r1;
//           orientation[2] = (m0[1] - m1[0]) * r1;
//       }
//       else if(( m0[0] >= m1[1]) && (m0[0] >= m2[2]) )
//       {
//           double r1 = sqrt(((1 + m0[0]) - m1[1]) - m2[2]);
//           double r2 = 0.5 / r1;
//           orientation[0] = 0.5 * r1;
//           orientation[1] = (m0[1] + m1[0]) * r2;
//           orientation[2] = (m0[2] + m2[0]) * r2;
//           orientation[3] = (m1[2] - m2[1]) * r2;
//       }
//       else if (m1[1] > m2[2])
//       {
//           double r1 = sqrt(((1 + m1[1]) - m0[0]) - m2[2]);
//           double r2 = 0.5 / r1;
//           orientation[0] = (m1[0] + m0[1]) * r2;
//           orientation[1] = 0.5 * r1;
//           orientation[2] = (m2[1] + m1[2]) * r2;
//           orientation[3] = (m2[0] - m0[2]) * r2;
//       }
//       else
//       {
//           double r1 = sqrt(((1 + m2[2]) - m0[0]) - m1[1]);
//           double r2 = 0.5 / r1;
//           orientation[0] = (m2[0] + m0[2]) * r2;
//           orientation[1] = (m2[1] + m1[2]) * r2;
//           orientation[2] = 0.5 * r1;
//           orientation[3] = (m0[1] - m1[0]) * r2;
//       }

//       //normalize quaternion
//       double mag = orientation[0] + orientation[1] + orientation[2] + orientation[3];
//       orientation[0] /= mag;
//       orientation[1] /= mag;
//       orientation[2] /= mag;
//       orientation[3] /= mag;

//     return orientation;
//   }

std::vector<double> rotationQuat(const gpd::GraspConfig& grasp, const std_msgs::Header& header){
  // for grasp[0]
//  if( grasps->grasps.size() == 0 )
//    return;
//std::cout << "HEADER2: " << getGraspSrv.response.grasps.header << std::endl;

  // // plot grasp base
  // visualization_msgs::Marker marker;
  // marker.header.frame_id = "/test";
  // marker.header.stamp = ros::Time(0);
  // marker.type = visualization_msgs::Marker::SPHERE;
  // marker.id = 100;
  // marker.action = visualization_msgs::Marker::ADD;
  // marker.pose.position = grasp.bottom;
  // marker.pose.orientation.w = 1.0;
  // marker.scale.x = 0.05;
  // marker.scale.y = 0.05;
  // marker.scale.z = 0.05;
  // marker.color.r = 0;
  // marker.color.g = 1;
  // marker.color.b = 0;
  // marker.color.a = 1;
  // marker_pub->publish(marker); // this works

  // // plot backoff point
  // marker.id = 101;
  // marker.color.r = 1;
  // marker.color.g = 0;

  // // backoff point generation
  // marker.pose.position.x = grasp.bottom.x - 0.08*grasp.approach.x;
  // marker.pose.position.y = grasp.bottom.y - 0.08*grasp.approach.y;
  // marker.pose.position.z = grasp.bottom.z - 0.08*grasp.approach.z;
  // marker_pub->publish(marker); // this works

  // determine orientation

  // publish grasp pose
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "/test";
  pose.header.stamp = ros::Time(0);
  pose.pose.position.x = grasp.bottom.x - 0.08*grasp.approach.x;
  pose.pose.position.y = grasp.bottom.y - 0.08*grasp.approach.y;
  pose.pose.position.z = grasp.bottom.z - 0.08*grasp.approach.z;

    Eigen::Vector3d up(grasp.binormal.x, grasp.binormal.y, grasp.binormal.z);
    Eigen::Vector3d forward(grasp.approach.x, grasp.approach.y, grasp.approach.z);

    Eigen::Vector3d m2 = forward;
    Eigen::Vector3d m0 = up.cross(forward);
    Eigen::Vector3d m1 = m2.cross(m0);

    double r0 = m0[0] + m1[1] + m2[2];
    if( r0 > 0 )
  {
        double r1 = sqrt(r0 + 1.0);
        pose.pose.orientation.w = r1 * 0.5;
        r1 = 0.5 / r1;
        pose.pose.orientation.x = (m1[2] - m2[1]) * r1;
        pose.pose.orientation.y = (m2[0] - m0[2]) * r1;
        pose.pose.orientation.z = (m0[1] - m1[0]) * r1;
    }
    else if(( m0[0] >= m1[1]) && (m0[0] >= m2[2]) )
    {
        double r1 = sqrt(((1 + m0[0]) - m1[1]) - m2[2]);
        double r2 = 0.5 / r1;
        pose.pose.orientation.x = 0.5 * r1;
        pose.pose.orientation.y = (m0[1] + m1[0]) * r2;
        pose.pose.orientation.z = (m0[2] + m2[0]) * r2;
        pose.pose.orientation.w = (m1[2] - m2[1]) * r2;
    }
    else if (m1[1] > m2[2])
    {
        double r1 = sqrt(((1 + m1[1]) - m0[0]) - m2[2]);
        double r2 = 0.5 / r1;
        pose.pose.orientation.x = (m1[0] + m0[1]) * r2;
        pose.pose.orientation.y = 0.5 * r1;
        pose.pose.orientation.z = (m2[1] + m1[2]) * r2;
        pose.pose.orientation.w = (m2[0] - m0[2]) * r2;
    }
    else
    {
        double r1 = sqrt(((1 + m2[2]) - m0[0]) - m1[1]);
        double r2 = 0.5 / r1;
        pose.pose.orientation.x = (m2[0] + m0[2]) * r2;
        pose.pose.orientation.y = (m2[1] + m1[2]) * r2;
        pose.pose.orientation.z = 0.5 * r1;
        pose.pose.orientation.w = (m0[1] - m1[0]) * r2;
    }

    //normalize quaternion
    //normalize quaternion
    double mag = sqrt(pose.pose.orientation.x*pose.pose.orientation.x + (pose.pose.orientation.y*pose.pose.orientation.y) +
      (pose.pose.orientation.z*pose.pose.orientation.z) + (pose.pose.orientation.w*pose.pose.orientation.w));
    pose.pose.orientation.x /= mag;
    pose.pose.orientation.y /= mag;
    pose.pose.orientation.z /= mag;
    pose.pose.orientation.w /= mag;

  // pose_pub->publish(pose);
  std::vector<double> orientation = {pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z};
  return orientation;
}


// ***********************************************

// TODO_AAMAS: ADDED FROM BAXTER_INTEGRATION BRANCH FROM MARIYA

bool checkOrientation (std::vector<double> orientation){
  Eigen::Quaternion<float> ori(orientation[0], orientation[1], orientation[2], orientation[3]);
  Eigen::Quaternion<float>::Matrix3 RotMat = ori.toRotationMatrix();
  Eigen::Vector3f unitVec(1,0,0);

  Eigen::Vector3f new_vect = RotMat*unitVec;
  std::cout << new_vect << std::endl;

  // now check if vector is close to (0,0,1) i.e. pointed down?

  float min_ori = 0.2;
  float max_ori = 1.7;

  if( fabs(new_vect[2]) < max_ori && fabs(new_vect[2]) > min_ori  ){
    std::cout << fabs(new_vect[2]) << " is between " << max_ori << " and " << min_ori << "\n";
    ROS_ERROR("ERROR: GRIPPER IS SIDEWAYS!" );
    // std::exit(1);
    return false;
  }
  std::cout << fabs(new_vect[2]) << " is NOT between " << max_ori << " and " << min_ori << "\n";
  ROS_WARN("NOTE: GRIPPER IS TOP DOWN!" );
  // std::exit(0);

  return true;
}

// ***********************************************


//----------------------------------------------------------------------
// vision manip pipeline
//----------------------------------------------------------------------
void vision_manip_pipeline_fxn(vision_manip_pipeline::VisionManip::Request &req,
                          vision_manip_pipeline::VisionManip::Response &res) {

    // set the default score to 0, which will be returned if any errors come up!
    res.score = std_msgs::Float32();
    res.score.data = 0.0;
    // res.grasp = getGraspSrv.response.grasps.grasps[indx]; // shouldn't need a grasp if location is bad?
    res.approach_pose = geometry_msgs::PoseStamped();
    res.approach_pose.pose.position.x = 10;
    res.approach_pose.pose.position.y = 10;
    res.approach_pose.pose.position.z = 10;
    res.pick_pose = geometry_msgs::PoseStamped();
    res.pick_pose.pose.position.x = 10;
    res.pick_pose.pose.position.y = 10;
    res.pick_pose.pose.position.z = 10;
    //TODO_AAMAS: Note, these values will cause moveit to fail soooo gotta fix that.....


    std::vector<double> cube(6);
    cube = {0,0,0,0,0,0};
    ros::param::set("/detect_grasps/workspace", cube);
    ros::param::set("/detect_grasps/workspace_grasps", cube);

    // now, do the vision manip stuff

    // --------------------------------------------
    // TODO: Replace below with Pourya's vision stuff
    // --------------------------------------------

    // // t = new tf::TransformListener();
    // // ros::Duration(1.0).sleep();
    // // ros::ServiceClient objLocClient = n.serviceClient<vision_manip_pipeline::GetObjLoc>("get_object_loc");
    // vision_manip_pipeline::GetObjLoc objLocSrv;

    // objLocSrv.request.obj_name = req.obj_name;
    // if(objLocClient_pntr->call(objLocSrv)) {
    //    std::cout << objLocSrv.response << '\n';
    // }
    // else{
    //    ROS_ERROR("ERROR: Failed to call objLoc Service!" );
    // }

    // int x = (objLocSrv.response.xmax - objLocSrv.response.xmin)/2 + objLocSrv.response.xmin;
    // int y = (objLocSrv.response.ymax - objLocSrv.response.ymin)/2 + objLocSrv.response.ymin;
    // std::cout << "x: " << x << ' ' << "y: " << y << '\n';

    // // if object not detected, then exit?
    // if(x == 0 && y == 0) {
    //    ROS_INFO("Error: Object not detected, try again!");
    //    // return -1;
    //    return;
    // }

    // // ros::ServiceClient conv2DTo3DClient = n.serviceClient<vision_manip_pipeline::Conv2DTo3D>("conv_coord");
    // vision_manip_pipeline::Conv2DTo3D conv2DTo3DSrv;

    // conv2DTo3DSrv.request.x = x;
    // conv2DTo3DSrv.request.y = y;
    // if(conv2DTo3DClient_pntr->call(conv2DTo3DSrv)) {
    //    std::cout << conv2DTo3DSrv.response << '\n';
    // }
    // else{
    //    ROS_ERROR("ERROR: Failed to call convCoord Service!" );
    // }

    // // transform the point from kinect frame to the orthographic frame (static tf projection)
    // geometry_msgs::PointStamped newPnt = transPoint(conv2DTo3DSrv.response.newX, conv2DTo3DSrv.response.newY, conv2DTo3DSrv.response.newZ, "/head_mount_kinect_rgb_optical_frame", "/test");
    // --------------------------------------------

    // --------------------------------------------
    // TODO: call Pourya's vision stuff
    // --------------------------------------------

    // t = new tf::TransformListener();
    // ros::Duration(1.0).sleep();
    // ros::ServiceClient pouryaClient = n.serviceClient<active_vision_msgs::Vision_Service>("vision_service");
    active_vision_msgs::Vision_Service pouryaSrv;

    pouryaSrv.request.Label = req.obj_name;
    if(pouryaClient_pntr->call(pouryaSrv)) {
       std::cout << pouryaSrv.response << '\n';
    }
    else{
       ROS_ERROR("ERROR: Failed to call objLoc Service!" );
    }

    // if object not detected, then exit?
    if(pouryaSrv.response.object_location.empty() || !pouryaSrv.response.object_location[0].Found ) {
       ROS_INFO("Error: Object not detected, try again!");
       // return -1;
       return;
    }

    // transform the point from kinect frame to the orthographic frame (static tf projection)
    geometry_msgs::PointStamped newPnt = transPoint(pouryaSrv.response.object_location[0].Pos.x, pouryaSrv.response.object_location[0].Pos.y, pouryaSrv.response.object_location[0].Pos.z, "/kinect2_rgb_optical_frame", "/test");
    // --------------------------------------------
    ROS_INFO("X: %lf, Y: %lf, Z: %lf", pouryaSrv.response.object_location[0].Pos.x, pouryaSrv.response.object_location[0].Pos.y, pouryaSrv.response.object_location[0].Pos.z);
    //std::exit(1);

    // point offsets - AAMAS- BAXTER only
    newPnt.point.x += double(0.14);
    newPnt.point.y -= double(0.40);
    newPnt.point.z -= double(0.16);
    //newPnt.point.y -= double(0.075);


    // generate the cube from the transformed point instead
    double eps = 0.075;

    // make sure in bounds
    bool inBounds = false;

    if( !checkInBounds(newPnt) ) {
      std::cout << "Error: Grasp outside of reachable arm space, will now return\n";
      // return -1;
      ROS_WARN("Since no grasp generated, will use vision location as grasp pose.");
      std::vector<double> fakeOri(4);
      fakeOri = {0,0,0,1};
      geometry_msgs::PoseStamped fakeApproach = getPoseTrans(newPnt.point.x, newPnt.point.y, newPnt.point.z, fakeOri, "/test", "/base");
      geometry_msgs::PoseStamped fakePick = getPoseTrans(newPnt.point.x, newPnt.point.y, newPnt.point.z, fakeOri, "/test", "/base");
      res.approach_pose = fakeApproach;
      res.pick_pose = fakePick;
       return;
    }

    // std::vector<double> cube(6);
    cube = {newPnt.point.x - eps, newPnt.point.x + eps, newPnt.point.y - eps, newPnt.point.y + eps, newPnt.point.z - 2*eps, 1.15};
   //cube = {newPnt.point.x - eps, newPnt.point.x + eps, newPnt.point.y - eps, newPnt.point.y + eps, newPnt.point.z - 2*eps, newPnt.point.z + 2*eps};

    std::cout << "cube to search for graps: " << cube[0] << ' ' << cube[1] << ' ' << cube[2] << ' ' << cube[3] << ' ' << cube[4] << ' ' << cube[5] << '\n';

    ros::param::set("/detect_grasps/workspace", cube);
    ros::param::set("/detect_grasps/workspace_grasps", cube);
    sleep(1);

    // ros::ServiceClient pubWorkspaceClient = n.serviceClient<vision_manip_pipeline::PubWorkspace>("pub_workspace_corners");
    vision_manip_pipeline::PubWorkspace pubWorkspaceSrv;

   /* std::vector<double> tempPos(3);
    tempPos = {0,0,0};
    std::vector<double> tempOri(4);
    tempOri = {0,0,0,0};

    pubWorkspaceSrv.request.pos = tempPos;
    pubWorkspaceSrv.request.pos2 = tempPos;
    pubWorkspaceSrv.request.ori = tempOri;*/
    if(pubWorkspaceClient_pntr->call(pubWorkspaceSrv)) {
       std::cout << pubWorkspaceSrv.response << '\n';
    }
    else{
       ROS_ERROR("ERROR: Failed to call pubWorkspace Service!" );
    }

    // ros::ServiceClient getGraspClient = n.serviceClient<vision_manip_pipeline::GetGrasp>("get_grasp");
    vision_manip_pipeline::GetGrasp getGraspSrv;

    pid_t pid;
    pid = fork();
    if(pid == 0) { // child process
        setpgid(getpid(), getpid());
        system("roslaunch vision_manip_pipeline jb_tutorial1.launch");
    }
    else {   // parent process

      // rosservice call to gpd with the calculated grasping window in the
      // point cloud to get the top grasp

      // --------------------------------------------
      // TODO: replace below with Pourya's vision stuff
      // --------------------------------------------
      // getGraspSrv.request.x = conv2DTo3DSrv.response.newX;
      // getGraspSrv.request.y = conv2DTo3DSrv.response.newY;
      // getGraspSrv.request.y = conv2DTo3DSrv.response.newZ;
      // --------------------------------------------

      // --------------------------------------------
      // TODO: call Pourya's vision stuff
      // --------------------------------------------
      getGraspSrv.request.x = pouryaSrv.response.object_location[0].Pos.x;
      getGraspSrv.request.y = pouryaSrv.response.object_location[0].Pos.y;
      getGraspSrv.request.y = pouryaSrv.response.object_location[0].Pos.z;
      // --------------------------------------------

      if(getGraspClient_pntr->call(getGraspSrv)) {
         std::cout << getGraspSrv.response << '\n';
      }
      else{
         ROS_ERROR("ERROR: Failed to call getGrasp Service!" );
      }

      kill(-pid, SIGKILL); // kill the launch process
      // signal(SIGINT, SIG_IGN); // kill the node it brings up?
      system("rosnode kill /detect_grasps");
      printf("killed process group %d\n", pid);
    }

  if( getGraspSrv.response.num_grasps == 0 ){
      std::cout << "Error: No grasp found, will now return\n";
      // return -1;
      ROS_WARN("Since no grasp generated, will use vision location as grasp pose.");
      std::vector<double> fakeOri(4);
      fakeOri = {0,0,0,1.0};
      geometry_msgs::PoseStamped fakeApproach = getPoseTrans(newPnt.point.x, newPnt.point.y, newPnt.point.z, fakeOri, "/test", "/base");
      geometry_msgs::PoseStamped fakePick = getPoseTrans(newPnt.point.x, newPnt.point.y, newPnt.point.z, fakeOri, "/test", "/base");
      res.approach_pose = fakeApproach;
      res.pick_pose = fakePick;
      return;
  }

  // in case none of the grasps generated are "good" grasps, default the approach and pick to the vision location
// ROS_WARN("Since no grasp generated, will use vision location as grasp pose.");
  std::vector<double> fakeOri(4);
  fakeOri = {0,0,0,1.0};
  geometry_msgs::PoseStamped fakeApproach = getPoseTrans(newPnt.point.x, newPnt.point.y, newPnt.point.z, fakeOri, "/test", "/base");
  geometry_msgs::PoseStamped fakePick = getPoseTrans(newPnt.point.x, newPnt.point.y, newPnt.point.z, fakeOri, "/test", "/base");
  res.approach_pose = fakeApproach;
  res.pick_pose = fakePick;

  // limit num grasp attempts to 10
  long int max_attempts = 50;
  int num_attempts = std::min(getGraspSrv.response.num_grasps, max_attempts);
  ROS_ERROR("THERE ARE %d grasps to search through", num_attempts);

  for(int indx = 0; indx < num_attempts; indx++) {

    // convert from geometry_msgs/Vector3 to std::vector<float>
    std::vector<double> axis(3);
    axis = {getGraspSrv.response.grasps.grasps[indx].axis.x, getGraspSrv.response.grasps.grasps[indx].axis.y, getGraspSrv.response.grasps.grasps[indx].axis.z};
    std::vector<double> binormal(3);
    binormal = {getGraspSrv.response.grasps.grasps[indx].binormal.x, getGraspSrv.response.grasps.grasps[indx].binormal.y, getGraspSrv.response.grasps.grasps[indx].binormal.z};
    std::vector<double> approach(3);
    approach = {getGraspSrv.response.grasps.grasps[indx].approach.x, getGraspSrv.response.grasps.grasps[indx].approach.y, getGraspSrv.response.grasps.grasps[indx].approach.z};

    // convert the top grasp format to a move-it useable format
    // then use moveit to move the arm of the Baxter/PR2? in rviz/eal world?
    std::cout << "HEADER: " << getGraspSrv.response.grasps.header << std::endl;
    std::vector<double> ori = rotationQuat(getGraspSrv.response.grasps.grasps[indx], getGraspSrv.response.grasps.header);
    std::cout << "ori: " << ori[0] << ',' << ori[1] << ',' << ori[2] << ',' << ori[3] << '\n';




    // ***********************************************
    // TODO_AAMAS: ADDED FROM BAXTER_INTEGRATION BRANCH FROM MARIYA - Add this in for AMAAS Only!

    // // rotate to vertical
    // tf::Quaternion q(ori[1], ori[2], ori[3], ori[0]);
    // // tf::Quaternion r(0,0,0,1);
    // // r.setEuler(0.,0.,1.5707963);
    // // q *= r;
    // std::cout << "test\n";
    // q.setRotation(q.getAxis(), q.getAngle() + 1.5707963);
    // q = q.normalize();
    // ori[0] = q.w();
    // ori[1] = q.x();
    // ori[2] = q.y();
    // ori[3] = q.z();

    // bool oriCheck = checkOrientation(ori);
    // ***********************************************


    // visualize the workspace and grasp.......
    std::vector<double> pos(3);

    std::vector<double> base(3);
    std::vector<double> vec(3);
    std::vector<double> ext_approach(3);
    std::vector<double> ext_pick(3);
    // pos = {getGraspSrv.response.grasps.grasps[indx].surface.x, getGraspSrv.response.grasps.grasps[indx].surface.y, getGraspSrv.response.grasps.grasps[indx].surface.z};

    // TODO: Try to plan to the approach point instead?!?!?!?! -> FIX THIS WITH DAVE MATH!!!!
    base = {getGraspSrv.response.grasps.grasps[indx].bottom.x, getGraspSrv.response.grasps.grasps[indx].bottom.y, getGraspSrv.response.grasps.grasps[indx].bottom.z};
    vec = {getGraspSrv.response.grasps.grasps[indx].approach.x, getGraspSrv.response.grasps.grasps[indx].approach.y, getGraspSrv.response.grasps.grasps[indx].approach.z};

    ext_pick = base;
    ext_pick[0] -= 0.01*vec[0];
    ext_pick[1] -= 0.01*vec[1];
    ext_pick[2] -= 0.01*vec[2];

    ext_approach = base;
    ext_approach[0] -= 0.05*vec[0];
    ext_approach[1] -= -0.075*vec[1];
    ext_approach[2] -= 0.05*vec[2];

    // pos = {getGraspSrv.response.grasps.grasps[indx].approach.x, getGraspSrv.response.grasps.grasps[indx].approach.y, getGraspSrv.response.grasps.grasps[indx].approach.z};
    pos = ext_pick;
    std::vector<double> tilt(4);
    tilt = {ori[0], ori[1], ori[2], ori[3]}; //TODO: double check order of w,x,y,z!
    std::cout << "pos: " << pos[0] << ',' << pos[1] << ',' << pos[2] << '\n';
    std::cout << "tilt: " << tilt[0] << ',' << tilt[1] << ',' << tilt[2] << ',' << tilt[3] << '\n';

    pubWorkspaceSrv.request.pos = ext_approach;
    pubWorkspaceSrv.request.pos2 = ext_pick;
    pubWorkspaceSrv.request.ori = tilt;
    if(pubWorkspaceClient_pntr->call(pubWorkspaceSrv)) {
       std::cout << pubWorkspaceSrv.response << '\n';
    }
    else{
       ROS_ERROR("ERROR: Failed to call pubWorkspace Service!" );
    }


    // // ***********************************************
    // // TODO_AAMAS: ADDED FROM BAXTER_INTEGRATION BRANCH FROM MARIYA - Add this in for AMAAS Only!

    // // rotate to vertical
    // tf::Quaternion q(ori[1], ori[2], ori[3], ori[0]);
    // // tf::Quaternion r(0,0,0,1);
    // // r.setEuler(0.,0.,1.5707963);
    // // q *= r;
    // std::cout << "test\n";
    // q.setRotation(q.getAxis(), q.getAngle() + 1.5707963);
    // q = q.normalize();
    // ori[0] = q.w();
    // ori[1] = q.x();
    // ori[2] = q.y();
    // ori[3] = q.z();

    bool oriCheck = checkOrientation(ori);
    // // ***********************************************



    // Transform point into correct PR2 frame for motion planning etc...
    geometry_msgs::PoseStamped newApproach = getPoseTrans(ext_approach[0], ext_approach[1], ext_approach[2], ori, "/test", "/base");
    geometry_msgs::PoseStamped newPick = getPoseTrans(ext_pick[0], ext_pick[1], ext_pick[2], ori, "/test", "/base");

    // std::cout << "final pos: " << newPose.pose.position.x << ',' << newPose.pose.position.y << ',' << newPose.pose.position.z << '\n';
    // std::cout << "final ori: " << newPose.pose.orientation.w << ',' << newPose.pose.orientation.x << ',' << newPose.pose.orientation.y << ',' << newPose.pose.orientation.z << '\n';

    // use moveit to plan to this position and orientation!
    // if ( moveArm(newApproach, newPick) ) {

    // // ***********************************************
    // // TODO_AAMAS: ADDED FROM BAXTER_INTEGRATION BRANCH FROM MARIYA - use this for AAMAS Only instead of the if move arm above
    if ( oriCheck && moveArm(newApproach, newPick) ) {
    // // ***********************************************

      // if successful grasp, set data to the response and return
      res.approach_pose = newApproach;
      res.pick_pose = newPick;
      res.score = getGraspSrv.response.grasps.grasps[indx].score;
      res.grasp = getGraspSrv.response.grasps.grasps[indx];
      break;
    }

  }

}


//----------------------------------------------------------------------
// handle for server
//----------------------------------------------------------------------
bool handle_vision_manip(vision_manip_pipeline::VisionManip::Request &req,
                        vision_manip_pipeline::VisionManip::Response &res){

    vision_manip_pipeline_fxn(req, res);
    return true;
}

//----------------------------------------------------------------------
// main
//----------------------------------------------------------------------
int main(int argc, char** argv){
  ros::init(argc, argv, "vision_manip");
  ros::NodeHandle n;

  ros::AsyncSpinner spinner(1);
  spinner.start();


  t = new tf::TransformListener();

  // advertise the service
  ros::ServiceServer service = n.advertiseService("vision_manip", handle_vision_manip);

  // ros::ServiceClient conv2DTo3DClient = n.serviceClient<vision_manip_pipeline::Conv2DTo3D>("conv_coord");
  // ros::ServiceClient objLocClient = n.serviceClient<vision_manip_pipeline::GetObjLoc>("get_object_loc");
  ros::ServiceClient pubWorkspaceClient = n.serviceClient<vision_manip_pipeline::PubWorkspace>("pub_workspace_corners");
  ros::ServiceClient getGraspClient = n.serviceClient<vision_manip_pipeline::GetGrasp>("get_grasp");
  // ros::ServiceClient pouryaClient = n.serviceClient<active_vision_msgs::Vision_Service>("vision_service");
  ros::ServiceClient pouryaClient = n.serviceClient<active_vision_msgs::Vision_Service>("/CV_Objects");

  // conv2DTo3DClient_pntr = &conv2DTo3DClient;
  // objLocClient_pntr = &objLocClient;
  pubWorkspaceClient_pntr = &pubWorkspaceClient;
  getGraspClient_pntr = &getGraspClient;
  pouryaClient_pntr = &pouryaClient;

  ROS_INFO("Ready to run vision manip pipeline.");
  ros::spin();

  return 0;
}




//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------



// bool checkOrientation (std::vector<double> orientation){
//   Eigen::Quaternion<float> ori(orientation[0], orientation[1], orientation[2], orientation[3]);
//   Eigen::Vector3f eulerOri = ori.toRotationMatrix().eulerAngles(0,1,2);

//   std::cout << "ORIENTATION ANGLES: " << eulerOri << std::endl;
//   std::cout << "ORIENTATION ANGLES: " << eulerOri[0] << eulerOri[1] << eulerOri[2] << std::endl;

//   // if(eulerOri[0] < -0.02 || eulerOri[0] > 1.4 || eulerOri[1] < -0.6 || eulerOri[1] > 0.51){
//   if( eulerOri[1] < 0.2 || eulerOri[1] > - 0.2 ){
//     ROS_ERROR("ERROR: ORIENTATION IS NOT TOP DOWN!" );
//     return false;
//   }
//   return true;
// }


// int main(){

//   std::vector<double> orientation(4);
//   orientation = {0.461, 0.886, 0.048, -0.018}; // change this value for use!!!!
//   std::cout << "QUAT:" << orientation[0] << std::endl;

//   checkOrientation (orientation);

//   return 0;
// }
