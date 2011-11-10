/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan, Sachin Chitta */

#include "ompl_interface_ros/ompl_interface_ros.h"
#include <sstream>

ompl_interface_ros::OMPLInterfaceROS::OMPLInterfaceROS(const std::string &robot_description) : ompl_interface::OMPLInterface(), nh_("~")
{
    planning_scene_ = new planning_scene_ros::PlanningSceneROS(robot_description);
    planning_scene_ptr_.reset(planning_scene_);
    if (planning_scene_->isConfigured())
    {
        // read configs from param server
        std::vector<ompl_interface::PlannerConfigs> pconfig;
        if (configure(planning_scene_ptr_, pconfig))
        {
            plan_service_ = nh_.advertiseService("plan_kinematic_path", &OMPLInterfaceROS::computePlan, this);
        }
    }
}

bool ompl_interface_ros::OMPLInterfaceROS::computePlan(moveit_msgs::GetMotionPlan::Request &req, moveit_msgs::GetMotionPlan::Response &res)
{
    return solve(req, res);
}

void ompl_interface_ros::OMPLInterfaceROS::run(void)
{
    if (isConfigured())
    {
        std::stringstream ss;
        planning_scene_->getKinematicModel()->printModelInfo(ss);
        ROS_INFO("%s", ss.str().c_str());
        ROS_INFO("OMPL planning node started.");
    }
    else
    {
        ROS_ERROR("Cannot start OMPL planning node.");
    }
}
