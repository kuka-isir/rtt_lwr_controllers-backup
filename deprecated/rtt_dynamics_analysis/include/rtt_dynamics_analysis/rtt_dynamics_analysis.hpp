// Copyright ISIR 2015
// Author: Antoine Hoarau <hoarau.robotics@gmail.com>

#ifndef __RTT_DYNAMICS_ANALYSIS_HPP__
#define __RTT_DYNAMICS_ANALYSIS_HPP__

#include <rtt/RTT.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Component.hpp>
#include <rtt_rosparam/rosparam.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt/os/TimeService.hpp>
#include <rtt/Logger.hpp>
#include <rtt_roscomm/rtt_rostopic.h>

#include <rtt_ros_kdl_tools/tools.hpp>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>

#include <tf_conversions/tf_kdl.h>

#include <Eigen/Dense>

#include <vector>
#include <boost/circular_buffer.hpp>
#include <boost/foreach.hpp>
#include <boost/assign.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

#ifdef CONMAN
#include <conman/conman.h>
#include <conman/scheme.h>
#include <conman/hook.h>
#endif

class DynamicsAnalysis : public RTT::TaskContext{
public:
    DynamicsAnalysis(const std::string& name);
    virtual ~DynamicsAnalysis(){};
    void updateHook();
    bool configureHook();
    bool use_robot_description;
    KDL::Chain chain;
    KDL::Tree tree;
    std::string root_link,tip_link,robot_name;
    std::string robot_description;
    boost::shared_ptr<KDL::ChainDynParam> id_dyn_solver;
    boost::shared_ptr<KDL::ChainIdSolver_RNE> id_rne_solver;
    boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver;
    boost::scoped_ptr<KDL::ChainIkSolverVel_pinv_nso> ik_solver_vel;
    
    sensor_msgs::JointState js_dyn_param,js_dyn,js;
    RTT::OutputPort<sensor_msgs::JointState> port_JSDynParam,port_JSDyn,port_JS;
    RTT::InputPort<Eigen::VectorXd> port_JointPosition,
                                    port_JointVelocity;
    RTT::OutputPort<Eigen::VectorXd> port_JointTorqueCommand;    
    KDL::Wrenches f_ext;
    KDL::JntArray gravity,q,qdot,qddot,jnt_trq_kdl,coriolis;
    KDL::JntSpaceInertiaMatrix mass;
    unsigned int n_joints_;
    
    Eigen::VectorXd jnt_pos,
                    jnt_vel,
                    jnt_vel_old,
                    jnt_acc,
                    jnt_pos_old,
                    jnt_trq,
                    inertia,
                    jnt_trq_cmd,
                    jnt_vel_cmd,
                    jnt_pos_cmd;
                    
    double amplitude,omega,phi,kp,kd,kg;
    unsigned int cnt,throttle;
    ros::Time now;
    
    
};

ORO_CREATE_COMPONENT(DynamicsAnalysis)
#endif
