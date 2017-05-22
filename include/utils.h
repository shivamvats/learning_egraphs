#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <boost/filesystem.hpp>
#include <fstream>

#include <ros/ros.h>
#include <smpl/robot_model.h>
#include <smpl/types.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>

namespace smpl = sbpl::motion;

bool writePath(const moveit_msgs::RobotState&,
        const moveit_msgs::RobotTrajectory& traj,
        smpl::RobotModel* robotModel,
        std::string plan_output_dir);

void convertJointVariablePathToJointTrajectory(
        const std::vector<smpl::RobotState>& path,
        trajectory_msgs::JointTrajectory& traj,
        smpl::RobotModel* robotModel,
        std::string planning_frame);

#endif
