// Copyright 2015 ISIR-CNRS
// Author: Antoine Hoarau

#ifndef __RTT_LWR_KRL_TOOL_HPP__
#define __RTT_LWR_KRL_TOOL_HPP__

#include <rtt/TaskContext.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Component.hpp>
#include <rtt_rosparam/rosparam.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt/os/TimeService.hpp>
#include <rtt/Logger.hpp>
#include <rtt_roscomm/rtt_rostopic.h>
#include <kuka_lwr_fri/friComm.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <lwr_fri/FriJointImpedance.h>

namespace lwr{
class KRLTool : public RTT::TaskContext{
public:
    KRLTool(const std::string& name);
    virtual ~KRLTool(){};
    void updateHook();
    bool configureHook();
    RTT::InputPort<tFriKrlData> port_fromKRL;
    RTT::OutputPort<tFriKrlData> port_toKRL;
    tFriKrlData toKRL;
    tFriKrlData fromKRL;
    RTT::InputPort<std_msgs::Int32MultiArray> port_intDataToKRL_ros;
    RTT::OutputPort<std_msgs::Int32MultiArray> port_intDataFromKRL_ros;
    std_msgs::Int32MultiArray intDataToKRL;
    std_msgs::Int32MultiArray intDataFromKRL;
    RTT::InputPort<std_msgs::Float32MultiArray> port_realDataToKRL_ros;
    RTT::OutputPort<std_msgs::Float32MultiArray> port_realDataFromKRL_ros;
    std_msgs::Float32MultiArray realDataToKRL;
    std_msgs::Float32MultiArray realDataFromKRL;
    bool send_diag;
    bool setJointImpedanceMode();
    bool setJointPositionMode();
    bool isJointPositionMode();
    bool isJointImpedanceMode();
    RTT::OutputPort<lwr_fri::FriJointImpedance> port_JointImpedanceCommand;
    void resetJointImpedanceGains();
    void setStiffnessZero();
};
}
ORO_CREATE_COMPONENT(lwr::KRLTool)
#endif
