#include "active_3d_planning_ros/planner/ros_planner.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <map>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>

#include "active_3d_planning_ros/module/module_factory_ros.h"
#include "active_3d_planning_ros/tools/ros_conversion.h"

#include "active_3d_planning_core/module/trajectory_generator/rrt_star.h"

namespace active_3d_planning {
namespace ros {

RosPlanner::RosPlanner(const ::ros::NodeHandle& nh,
                       const ::ros::NodeHandle& nh_private,
                       ModuleFactory* factory, Module::ParamMap* param_map)
    : OnlinePlanner(factory, param_map), nh_(nh), nh_private_(nh_private) {
  // params
  RosPlanner::setupFromParamMap(param_map);
  perf_log_data_ = std::vector<double>(
      6, 0.0);  // select, expand, gain, cost, value, mainLoop, rosCallbacks

  // Subscribers and publishers
  target_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      "command/trajectory", 10);
  trajectory_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "trajectory_visualization", 100);
  odom_sub_ = nh_.subscribe("odometry", 1, &RosPlanner::odomCallback, this);
  get_cpu_time_srv_ = nh_private_.advertiseService(
      "get_cpu_time", &RosPlanner::cpuSrvCallback, this);

  // ROS oneshot timer
  timeout_timer = nh.createTimer(::ros::Duration(1.0), &RosPlanner::timerCallback, this, true);
  timeout_timer.stop();

  // setup waypoints from txt file
  std::string line;   // To read each line from code
  int i=0;    // Variable to keep count of each line
  std::ifstream mFile("/home/students/AORACLE_baseline/vsep/vsep_sparse5/waypoint_gallery_Explore.txt");   
  std::cout << "WAYPOINT:" << std::endl;
  if(mFile.is_open())
  {
    while(!mFile.eof())
    {
      getline(mFile, line);
      // parse the line
      std::stringstream streamData(line);
      std::vector<std::string> waypoint_coordinates;
      std::string tmp_string;
      while (getline(streamData, tmp_string, ' ')) {
        waypoint_coordinates.push_back(tmp_string);
      }
      if (waypoint_coordinates.size() > 0) {
        double x = std::stod(waypoint_coordinates[0]);
        double y = std::stod(waypoint_coordinates[1]);
        double z = std::stod(waypoint_coordinates[2]);
        // print
        std::cout << x << "," << y << "," << z << std::endl;
        // push to the waypoint list
        waypoints.push_back(Eigen::Vector3d(x, y, z));
        i++;
      }
    }
    mFile.close();
  }

  R = 1;
  waypoint_idx = 0;
  current_waypoint = waypoints[waypoint_idx];

  // Finish
  ROS_INFO_STREAM(
      "\n******************** Initialized Planner ********************\n"
      "Initialized 'RosPlanner' from namespace '"
      << param_map->at("param_namespace")
      << "' with parameters:" << param_map->at("verbose_text"));
}

void RosPlanner::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<double>(param_map, "replan_pos_threshold", &p_replan_pos_threshold_,
                   0.1);
  setParam<double>(param_map, "replan_yaw_threshold", &p_replan_yaw_threshold_,
                   0.1);
}

void RosPlanner::setupFactoryAndParams(ModuleFactory* factory,
                                       Module::ParamMap* param_map,
                                       const ::ros::NodeHandle& nh_private) {
  // setup module factory
  factory = dynamic_cast<ModuleFactoryROS*>(factory);
  if (!factory) {
    delete factory;
    factory = new ModuleFactoryROS();
  }

  // setup params
  *param_map = Module::ParamMap();
  std::string type;
  const std::string& args = nh_private.getNamespace();
  factory->getParamMapAndType(param_map, &type, args);
  param_map->at("verbose_text") = "";
}

void RosPlanner::initializePlanning() {
  // setup standard
  OnlinePlanner::initializePlanning();

  // Update performance log for simulated time and ros cpu consumption
  if (p_log_performance_) {
    perf_cpu_timer_ = std::clock();
    perf_log_file_ << ",RosTime,RosCallbacks";
  }

  // Setup counters
  cpu_srv_timer_ = std::clock();
  ros_timing_ = ::ros::Time::now();
  perf_log_data_[5] = 0;  // reset count
}

void RosPlanner::planningLoop() {
  // This is the main loop, spinning is managed explicitely for efficiency
  ROS_INFO(
      "\n******************** Planner is now Running ********************\n");
  run_srv_ = nh_private_.advertiseService("toggle_running",
                                          &RosPlanner::runSrvCallback, this);

  VisualizationMarkers vis_waypoints;
  for (size_t i = 0; i < waypoints.size(); i++) {
    VisualizationMarker marker;
    marker.position = waypoints[i];
    marker.scale = Eigen::Vector3d(R, R, R);
    marker.type = VisualizationMarker::SPHERE;
    marker.color.r = 1.;
    marker.color.g = 0.;
    marker.color.b = 0.;
    marker.color.a = 0.3;
    marker.id = 2*i;
    vis_waypoints.addMarker(marker);
    std::cout << "added marker to vis list: " << marker.position.transpose() << std::endl;

    marker.type = VisualizationMarker::TEXT_VIEW_FACING;
    marker.position(2) += 1.0;
    marker.color.r = 0.;
    marker.color.g = 1.;
    marker.color.b = 0.;    
    marker.color.a = 1.0;
    marker.id = 2*i + 1;
    marker.text = std::to_string(i+1);
    vis_waypoints.addMarker(marker);
  }
  visualization_msgs::MarkerArray waypoint_msg;
  visualizationMarkersToMsg(vis_waypoints, &waypoint_msg);
  for (visualization_msgs::Marker& m : waypoint_msg.markers) {
    m.header.frame_id = "world";
    m.header.stamp = ::ros::Time::now();
  }
  ::ros::Publisher waypoint_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "waypoint_visualization", 100);

  running_ = true;
  std::clock_t timer;
  
  int waypoint_pub_cnt = 0;
  
  while (::ros::ok()) {
    if (planning_) {
      loopIteration();
    }
    waypoint_pub_cnt++;
    if (waypoint_pub_cnt == 100) {
      waypoint_vis_pub_.publish(waypoint_msg);
      waypoint_pub_cnt = 0;
    }
    if (p_log_performance_) {
      timer = std::clock();
    }
    ::ros::spinOnce();
    if (p_log_performance_) {
      perf_log_data_[5] +=
          static_cast<double>(std::clock() - timer) / CLOCKS_PER_SEC;
    }
  }
}

bool RosPlanner::requestNextTrajectory() {
  // call standard procedure
  if (!OnlinePlanner::requestNextTrajectory()) {
    return false;
  }

  // Performance log
  if (p_log_performance_) {
    perf_log_file_ << "," << (::ros::Time::now() - ros_timing_).toSec() << ","
                   << perf_log_data_[5];
    perf_cpu_timer_ = std::clock();
    ros_timing_ = ::ros::Time::now();
    perf_log_data_[5] = 0;  // reset count
  }
  return true;
}

void RosPlanner::odomCallback(const nav_msgs::Odometry& msg) {
  // Track the current pose
  current_position_ =
      Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y,
                      msg.pose.pose.position.z);
  current_orientation_ = Eigen::Quaterniond(
      msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
  if (running_ && !target_reached_) {
    // check goal pos reached (if tol is set)
    if (p_replan_pos_threshold_ <= 0 ||
        (target_position_ - current_position_).norm() <
            p_replan_pos_threshold_) {
      // check goal yaw reached (if tol is set)
      double yaw = tf::getYaw(msg.pose.pose.orientation);
      if (p_replan_yaw_threshold_ <= 0 ||
          defaults::angleDifference(target_yaw_, yaw) <
              p_replan_yaw_threshold_) {
        target_reached_ = true;
      }
    }
  }
  Eigen::Vector2d current_xy = current_position_.segment(0, 2);
  Eigen::Vector2d target_xy = current_waypoint.segment(0, 2);
  Eigen::Vector2d diff_xy = current_xy - target_xy;
  if (running_ && diff_xy.norm() < R) {
    waypoint_idx++;
    if (waypoint_idx >= waypoints.size()) {
      std::cout << "Finished all waypoints - stopping run" << std::endl;
      running_ = false;
    }
    else {
      double timeout_period;
      Eigen::Vector3d old_waypoint = current_waypoint;
      do {
        std::cout << "Waypoint " << waypoint_idx << " reached. Moving on to waypoint " << waypoint_idx+1 << std::endl;
        current_waypoint = waypoints[waypoint_idx];
        // start ros timer here (no need for first waypoint with time = distance * factor / vel (0.75 m/s)
        timeout_timer.stop();
        timeout_period = (waypoints[waypoint_idx] - old_waypoint).norm() * 2.0 / getSystemConstraints().v_max;
        if (timeout_period == 0) {
          waypoint_idx++;
        }
      } while (timeout_period == 0);
      timeout_timer.setPeriod(::ros::Duration(timeout_period));
      timeout_timer.start();
    }
  }
}

// timeout callback -> print something -> send MPC traj to go to the next wp
void RosPlanner::timerCallback(const ::ros::TimerEvent& event)
{
  std::cout << "TIMEOUT while going to waypoint " << waypoint_idx+1 << std::endl;

  // create mew trajectory
  EigenTrajectoryPointVector trajectory;
  EigenTrajectoryPoint tmp_point, tmp_point_2;
  tmp_point.position_W = current_position_;
  tmp_point.orientation_W_B = current_orientation_;
  tmp_point.time_from_start_ns = 0;
  trajectory.push_back(tmp_point);
  
  double yaw_to_next_waypoint;
  if (waypoint_idx >= 1) {
    Eigen::Vector3d waypoint_vec = current_waypoint - current_position_;
    if ((waypoint_vec[0] != 0) || (waypoint_vec[1] != 0)) {
      yaw_to_next_waypoint = atan2(waypoint_vec[1], waypoint_vec[0]);
    }
    else {
      yaw_to_next_waypoint = 0.0;
    }
  }
  else { // shouldn't go here
    yaw_to_next_waypoint = 0.0;
  }

  // change the yaw angle to point to the next waypoint (clear the unknown space)
  tmp_point.setFromYaw(yaw_to_next_waypoint);
  tmp_point.time_from_start_ns = 3e9; // can rotate at least 180 degrees
  trajectory.push_back(tmp_point);

  // go to the current target in straight line
  tmp_point_2 = tmp_point;
  tmp_point_2.position_W = target_position_;
  tmp_point_2.setFromYaw(target_yaw_);
  tmp_point_2.time_from_start_ns += (target_position_ - current_position_).norm() / getSystemConstraints().v_max * 1e9;
  trajectory.push_back(tmp_point_2);

  EigenTrajectoryPointVector result;
  // go to the current target in straight line if it's non-collision connection
  trajectory_generator::RRTStar* rrt_star_ptr = static_cast<trajectory_generator::RRTStar*>(trajectory_generator_.get()); // DIRTY HACK 
  if (rrt_star_ptr->connectPoses(tmp_point, tmp_point_2,
                     &result, true)) { // check for collision

    requestMovement(trajectory);
  }
}

void RosPlanner::requestMovement(const EigenTrajectoryPointVector& trajectory) {
  if (trajectory.empty()) {
    LOG(WARNING) << "Tried to publish an empty trajectory";
    return;
  }
  trajectory_msgs::MultiDOFJointTrajectoryPtr msg(
      new trajectory_msgs::MultiDOFJointTrajectory);
  msg->header.stamp = ::ros::Time::now();
  int n_points = trajectory.size();
  msg->points.resize(n_points);
  for (int i = 0; i < n_points; ++i) {
    msgMultiDofJointTrajectoryPointFromEigen(trajectory[i], &msg->points[i]);
  }
  target_pub_.publish(msg);
}

void RosPlanner::publishVisualization(const VisualizationMarkers& markers) {
  if (markers.getMarkers().empty()) {
    return;
  }
  visualization_msgs::MarkerArray msg;
  visualizationMarkersToMsg(markers, &msg);
  for (visualization_msgs::Marker& m : msg.markers) {
    m.header.frame_id = "world";
    m.header.stamp = ::ros::Time::now();
  }
  // check overwrite flag (for full array)
  if (markers.getMarkers()[0].action == VisualizationMarker::OVERWRITE) {
    for (visualization_msgs::Marker& m : msg.markers) {
      m.action = visualization_msgs::Marker::ADD;
    }
    std::map<std::string, int>::iterator it =
        visualization_overwrite_counter_.find(markers.getMarkers()[0].ns);
    int count = 0;
    if (it != visualization_overwrite_counter_.end()) {
      count = it->second;
    }
    if (count > markers.getMarkers().size()) {
      visualization_msgs::Marker marker;
      for (int i = markers.getMarkers().size(); i < count; ++i) {
        // publish empty marker to remove previous ids
        auto empty_marker = visualization_msgs::Marker();
        empty_marker.header.frame_id = "world";
        empty_marker.header.stamp = ::ros::Time::now();
        empty_marker.type = visualization_msgs::Marker::POINTS;
        empty_marker.id = i;
        empty_marker.ns = markers.getMarkers()[0].ns;
        empty_marker.action = visualization_msgs::Marker::DELETE;
        msg.markers.push_back(empty_marker);
      }
    }
    visualization_overwrite_counter_[markers.getMarkers()[0].ns] =
        markers.getMarkers().size();
  }
  trajectory_vis_pub_.publish(msg);
}

bool RosPlanner::runSrvCallback(std_srvs::SetBool::Request& req,
                                std_srvs::SetBool::Response& res) {
  res.success = true;
  if (req.data) {
    initializePlanning();
    planning_ = true;
    ROS_INFO("Started planning.");
  } else {
    planning_ = false;
    ROS_INFO("Stopped planning.");
  }
  return true;
}

bool RosPlanner::cpuSrvCallback(std_srvs::SetBool::Request& req,
                                std_srvs::SetBool::Response& res) {
  double time =
      static_cast<double>(std::clock() - cpu_srv_timer_) / CLOCKS_PER_SEC;
  cpu_srv_timer_ = std::clock();

  // Just return cpu time as the service message
  res.message = std::to_string(time).c_str();
  res.success = true;
  return true;
}

// logging and printing
void RosPlanner::printInfo(const std::string& text) { ROS_INFO_STREAM(text); }

void RosPlanner::printWarning(const std::string& text) {
  ROS_WARN_STREAM(text);
}

void RosPlanner::printError(const std::string& text) { ROS_ERROR_STREAM(text); }

}  // namespace ros
}  // namespace active_3d_planning
