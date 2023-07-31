#include "active_3d_planning_ros/planner/ros_planner.h"

#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>

#include "active_3d_planning_ros/module/module_factory_ros.h"
#include "active_3d_planning_ros/tools/ros_conversion.h"

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

  // setup waypoints
  //waypoints.push_back(Eigen::Vector3d(2.5002, 9.50012, 0.74016)); // the first waypoint is the starting position
  waypoints.push_back(Eigen::Vector3d(-1.15879, 11.0427, 1.22193));
  waypoints.push_back(Eigen::Vector3d(-4.92934, 12.3464, 1.5105));
  waypoints.push_back(Eigen::Vector3d(-7.71492, 9.54075, 0.903292));
  waypoints.push_back(Eigen::Vector3d(-8.67476, 5.75681, 1.77536));
  waypoints.push_back(Eigen::Vector3d(-4.75464, 6.27141, 1.16884));
  waypoints.push_back(Eigen::Vector3d(-7.8862, 4.08217, 2.35229));
  waypoints.push_back(Eigen::Vector3d(-5.97726, 0.569703, 2.21613));
  waypoints.push_back(Eigen::Vector3d(-3.48569, 3.68987, 2.45425));
  waypoints.push_back(Eigen::Vector3d(-6.7687, 1.55481, 1.63976));
  waypoints.push_back(Eigen::Vector3d(-5.65648, -2.28509, 1.50499));
  waypoints.push_back(Eigen::Vector3d(-7.56109, -5.7875, 1.18002));
  waypoints.push_back(Eigen::Vector3d(-6.88733, -9.72227, 0.927783));
  waypoints.push_back(Eigen::Vector3d(-7.27365, -5.76098, 1.32646));
  waypoints.push_back(Eigen::Vector3d(-4.42727, -8.56255, 1.10445));
  waypoints.push_back(Eigen::Vector3d(-0.982985, -10.5619, 1.4782));
  waypoints.push_back(Eigen::Vector3d(-2.39502, -6.82154, 1.6053));
  waypoints.push_back(Eigen::Vector3d(1.28231, -8.36602, 1.90839));
  waypoints.push_back(Eigen::Vector3d(5.17287, -7.76231, 1.20194));
  waypoints.push_back(Eigen::Vector3d(1.31338, -8.31609, 2.09508));
  waypoints.push_back(Eigen::Vector3d(-2.66011, -8.15369, 1.665));
  waypoints.push_back(Eigen::Vector3d(-6.0572, -10.1666, 1.02613));
  waypoints.push_back(Eigen::Vector3d(-6.07122, -6.1938, 1.49188));
  waypoints.push_back(Eigen::Vector3d(-5.94604, -2.20287, 1.25357));
  waypoints.push_back(Eigen::Vector3d(-3.73392, 1.08076, 1.82308));
  waypoints.push_back(Eigen::Vector3d(-0.928438, -1.75279, 1.5064));
  waypoints.push_back(Eigen::Vector3d(0.0961273, 2.09549, 1.88198));
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
  while (::ros::ok()) {
    if (planning_) {
      loopIteration();
    }
    waypoint_vis_pub_.publish(waypoint_msg);
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
  if (running_ && (current_position_ - current_waypoint).norm() < R) {
    waypoint_idx++;
    if (waypoint_idx >= waypoints.size()) {
      std::cout << "Finished all waypoints - stopping run" << std::endl;
      running_ = false;
    }
    else {
      std::cout << "Waypoint " << waypoint_idx-1 << " reached. Moving on to waypoint " << waypoint_idx << std::endl;
      current_waypoint = waypoints[waypoint_idx];
    }
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
