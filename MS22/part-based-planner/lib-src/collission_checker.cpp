/*
 * collision_checker.h
 *
 *  Created on: 06.08.2014
 *      Author: alex
 *
 * Compute collsion check with full planning scene
 * Input is jnt model group and std vector joints, ouput bool and maybe some message telling where
 */
#include <pbp/collission_checker.hpp>

void collisionChecker::setJointGroup(const std::string & joint_group_name){
    joint_group_name_=joint_group_name;
    joint_group_ = state_->getJointModelGroup(joint_group_name_);
    if(joint_group_ == NULL) {
        ROS_ERROR("Unknown IK-Group '%s'", joint_group_name_.c_str());
        exit(EXIT_FAILURE);
    }
}

bool collisionChecker::is_colliding_scene(const std::vector<double> joints){
    state_->setJointGroupPositions(joint_group_, joints);
    state_->update();
    return planning_scene_->isStateColliding(*state_, joint_group_->getName());
}

collisionChecker::collisionChecker(const robot_model::RobotModelPtr & model)
{

    planning_scene_.reset(new planning_scene::PlanningScene(model));
    state_.reset(new robot_state::RobotState(model));
    state_->setToDefaultValues();
    setJointGroup("right_arm");

}
