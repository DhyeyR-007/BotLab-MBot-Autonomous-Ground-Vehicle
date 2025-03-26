#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.005f)
, k2_(0.005f)
, min_dist_(0.002)
, min_theta_(0.02)
, initialized_(false)
{
    //////////////// TODO(DONE): Handle any initialization for your ActionModel /////////////////////////
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd()); 

}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose2D_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose2D_t& odometry)
{
    ////////////// TODO(DONE): Implement code here to compute a new distribution of the motion of the robot ////////////////
    if(!initialized_){
        resetPrevious(odometry);
        initialized_ = true;
    }

    dx_ = odometry.x - previousPose_.x;
    dy_ = odometry.y - previousPose_.y; 
    dtheta_ = angle_diff(odometry.theta, previousPose_.theta);

    // Rotate - Translate - Rotate
    rot1_ = angle_diff(std::atan2(dy_, dx_), previousPose_.theta);
    theta_ = std::sqrt(dx_ * dx_ + dy_ * dy_);

    // if delta angle > PI/2, change direction (move backward) 
    int direction = 1;
    if (std::abs(rot1_) > M_PI_2)
    {
        rot1_ = angle_diff(M_PI, rot1_);
        direction = -1;
    }

    rot2_ = angle_diff(dtheta_, rot1_);
    bool if_move = (dx_ != 0) || (dy_ != 0) || (dtheta_ != 0);
    if (!if_move) {
            rot1Std_ = 0;
            thetaStd_ = 0;
            rot2Std_ = 0;
        } else {
            rot1Std_ = abs(k1_ * rot1_);
            thetaStd_ = abs(k2_ * theta_);
            rot2Std_ = abs(k1_ * rot2_);

        }

    theta_ *= direction;
    utime_ = odometry.utime;
    previousPose_ = odometry;

    return if_move;    // Placeholder
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO(DONE): Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample = sample;

    // Assume that cross correlation alpha2 = alpha4 = 0
    std::normal_distribution<double> rot1_sample(0.0, rot1Std_);
    std::normal_distribution<double> trans_sample(0.0, thetaStd_);
    std::normal_distribution<double> rot2_sample(0.0, rot2Std_);

    double rot1_hat = angle_diff(rot1_, rot1_sample(numberGenerator_));
    double trans_hat = theta_ - trans_sample(numberGenerator_);
    double rot2_hat = angle_diff(rot2_, rot2_sample(numberGenerator_));

    newSample.pose.x += trans_hat * std::cos(angle_sum(sample.pose.theta, rot1_hat));
    newSample.pose.y += trans_hat * std::sin(angle_sum(sample.pose.theta, rot1_hat));
    newSample.pose.theta = angle_sum(newSample.pose.theta, angle_sum(rot1_hat, rot2_hat));

    newSample.parent_pose = sample.pose;
    newSample.pose.utime = utime_;
    return newSample;
}
