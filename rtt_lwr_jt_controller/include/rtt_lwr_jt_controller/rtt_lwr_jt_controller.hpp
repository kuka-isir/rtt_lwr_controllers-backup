// rtt_lwr_jt_controller - ISIR Wed 24 Jun 2015 05:00:06 PM CEST
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

#ifndef __RTT_LWR_JT_CONTROLLER_HPP__
#define __RTT_LWR_JT_CONTROLLER_HPP__

#include "rtt_lwr_abstract/rtt_lwr_abstract.hpp"
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <algorithm>
#include <eigen_conversions/eigen_msg.h>
#include <kdl_conversions/kdl_msg.h>


namespace lwr{
  class JtATIController : public RTTLWRAbstract{
    public:
      JtATIController(const std::string& name);
      virtual ~JtATIController(){};
      void updateHook();
      bool configureHook();
      RTT::InputPort<geometry_msgs::WrenchStamped> port_ftdata;
      void setDamping(double d);
    protected:
      geometry_msgs::WrenchStamped wrench_msg;
      Eigen::Matrix<double,6,1> wrench;
      KDL::Jacobian J_kdl;
      std::string ati_frame;
      KDL::JntArray jnt_acc_kdl;
      KDL::Wrench wrench_kdl;
      KDL::JntSpaceInertiaMatrix mass_kdl;
      bool use_kdl;
      bool use_kdl_gravity;
      bool add_damping;
      bool compensate_coriolis;
      Eigen::VectorXd kg;
      Eigen::VectorXd kd;
      Eigen::Affine3d tool_in_base_frame_eigen;
  };
}
ORO_CREATE_COMPONENT(lwr::JtATIController)
#endif
