// Antoine Hoarau hoarau.robotics@gmail.com
// Copyright ISIR 2015
#include <rtt/Component.hpp>
#include <sensor_msgs/JointState.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <Eigen/Core>
#include <time.h>
#include <rtt/os/TimeService.hpp>
#include <sstream>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/Operation.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <urdf/model.h>
#include <rtt_roscomm/rtt_rostopic.h>
#include <std_msgs/Float32.h>
#include <rtt_rosparam/rosparam.h>
#include <rtt_ros_kdl_tools/tools.hpp>
#include <ros/param.h>

class RTTStatePublisher : public RTT::TaskContext{
public:
    RTTStatePublisher(std::string const& name):
        RTT::TaskContext(name)
    {
            addProperty("robot_description",robot_description);
            addPort("JointPosition", port_JointPosition).doc("");
            addPort("JointVelocity", port_JointVelocity).doc("");
            addPort("JointTorque", port_JointTorque).doc("");
            addPort("JointStates", port_JointStates).doc("");
    }

    virtual ~RTTStatePublisher(){}

    bool configureHook(){
        if(!ros::param::get("robot_description",robot_description))
            return false;

        if(! rtt_ros_kdl_tools::initJointStateMsgFromString(robot_description,joint_state)) return false;

        //RTT::log(RTT::Info) << "Joint state message : "<<joint_state<<RTT::endlog();

        unsigned int dof = joint_state.position.size();
        if(!dof) return false;

        jnt_pos.resize(dof);
        jnt_vel.resize(dof);
        jnt_trq.resize(dof);

        jnt_pos.setZero();
        jnt_vel.setZero();
        jnt_trq.setZero();

        port_JointStates.setDataSample(joint_state);
        return true;
    }

    void updateHook() {
        static const unsigned int dof = joint_state.position.size();

        jnt_pos_fs = port_JointPosition.readNewest(jnt_pos);
        jnt_vel_fs = port_JointVelocity.readNewest(jnt_vel);
        jnt_trq_fs = port_JointTorque.readNewest(jnt_trq);

        joint_state.header.stamp = rtt_rosclock::host_now();

        if(jnt_pos.size() != dof) return;
        if(jnt_vel.size() != dof) return;
        if(jnt_trq.size() != dof) return;

        if(jnt_pos_fs != RTT::NoData) Eigen::Map<Eigen::VectorXd>(joint_state.position.data(),dof) = jnt_pos;
        if(jnt_vel_fs != RTT::NoData) Eigen::Map<Eigen::VectorXd>(joint_state.velocity.data(),dof) = jnt_vel;
        if(jnt_trq_fs != RTT::NoData) Eigen::Map<Eigen::VectorXd>(joint_state.effort.data(),dof) = jnt_trq;

        if(joint_state.header.stamp.isZero() && (
            (jnt_pos_fs == RTT::NewData) ||
            (jnt_vel_fs == RTT::NewData) ||
            (jnt_trq_fs == RTT::NewData)))
        {
            RTT::log(RTT::Error)<<"rtt_rosclock::host_now() returned 0, please check if /clock is published"<<RTT::endlog();
            joint_state.header.stamp = rtt_rosclock::rtt_now();
        }
        port_JointStates.write(joint_state);
    }

protected:
    sensor_msgs::JointState joint_state;
    RTT::OutputPort<sensor_msgs::JointState> port_JointStates;
    RTT::InputPort<Eigen::VectorXd > port_JointPosition;
    RTT::InputPort<Eigen::VectorXd > port_JointVelocity;
    RTT::InputPort<Eigen::VectorXd > port_JointTorque;

    RTT::FlowStatus jnt_pos_fs,jnt_vel_fs,jnt_trq_fs;

    Eigen::VectorXd jnt_pos,jnt_vel,jnt_trq;

    std::string robot_description;
};

ORO_CREATE_COMPONENT(RTTStatePublisher)
