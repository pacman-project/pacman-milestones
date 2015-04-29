/*
 * collision_checker.h
 *
 *  Created on: 06.08.2014
 *      Author: alex
 *
 * Compute collsion check with full planning scene
 * Input is jnt model group and std vector joints, ouput bool and maybe some message telling where
 */
#ifndef COLLISSION_CHECKER_H
#define COLLISSION_CHECKER_H

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>

class collisionChecker{

private:
    planning_scene::PlanningScenePtr planning_scene_;
    robot_model::RobotStatePtr state_;
    const robot_model::JointModelGroup *joint_group_;
    std::vector<double> joint_pos_;
    std::string joint_group_name_;

public:

    void setJointGroup(const std::string & joint_group_name);
    bool is_colliding_scene(const std::vector<double> joints);
    collisionChecker(const robot_model::RobotModelPtr & model);
};

#endif
