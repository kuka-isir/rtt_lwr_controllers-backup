// rtt_demo_load_comp - ISIR Wed 24 Jun 2015 05:00:06 PM CEST
// Copyright (c) Antoine Hoarau, All rights reserved.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3.0 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library.

#ifndef __LOAD_COMPENSATION_DEMO_HPP__
#define __LOAD_COMPENSATION_DEMO_HPP__

#include <rtt_ros_kdl_tools/chain_utils.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Component.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <kdl_conversions/kdl_msg.h>

class LoadCompDemo : public RTT::TaskContext
{
public:
    LoadCompDemo(const std::string& name);
    virtual ~LoadCompDemo(){};
    void updateHook();
    bool configureHook();
    Eigen::VectorXd damping;

protected:

    RTT::InputPort<geometry_msgs::WrenchStamped> port_ftdata;

    RTT::InputPort<Eigen::VectorXd>  port_joint_position_in,
                                    port_joint_velocity_in;
    // Some input variables
    Eigen::VectorXd jnt_pos_in,
                    jnt_vel_in;
    // Output ports
    RTT::OutputPort<Eigen::VectorXd> port_joint_torque_cmd_out;
    // Some output variables
    Eigen::VectorXd jnt_trq_cmd_out;

protected:
    KDL::JntArray zero_kdl;
    geometry_msgs::WrenchStamped ft_msg;
    rtt_ros_kdl_tools::ChainUtils arm;
    Eigen::Matrix<double,6,1> wrench_eigen;
    KDL::Wrench wrench_kdl;
    bool add_damping;
    bool compensate_coriolis;
    Eigen::VectorXd kg;

    Eigen::Affine3d tool_in_base_frame_eigen;
    bool use_ft_sensor;
    std::string ft_sensor_link;
};

ORO_CREATE_COMPONENT(LoadCompDemo)
#endif
