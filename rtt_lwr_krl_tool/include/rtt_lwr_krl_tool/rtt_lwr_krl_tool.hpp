// Copyright 2015 ISIR-CNRS
// Author: Antoine Hoarau

#ifndef __RTT_LWR_KRL_TOOL_HPP__
#define __RTT_LWR_KRL_TOOL_HPP__

#include <rtt/TaskContext.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Logger.hpp>

#include <rtt_roscomm/rtt_rostopic.h>
#include <rtt_rosparam/rosparam.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>

#include <lwr_fri/FriJointImpedance.h>
#include <kuka_lwr_fri/friComm.h>
#include <rtt_lwr_krl_tool/fri_user_data_description.h>

template<typename T> static bool bitStatus(const T& in, unsigned int bit_number)
{
    return ((in >> bit_number) & 1);
}

template<typename T> static void setBit(T& in, unsigned int bit_number,bool status)
{
    if(bitStatus(in,bit_number))
        in &= ( (status ? 1:0) << bit_number);
    else
        in |= ( (status ? 1:0) << bit_number);
//     in ^= (-status ^ in) & (1 << bit_number);
    return;
}

namespace lwr{
static const int ROS_MASK_NO_UPDATE = -9999;

class KRLTool : public RTT::TaskContext{
public:
    KRLTool(const std::string& name);
    virtual ~KRLTool(){};
    void updateHook();
    bool configureHook();
protected:
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
    RTT::OutputPort<std_msgs::ByteMultiArray> port_boolDataFromKRL_ros;
    std_msgs::ByteMultiArray boolDataFromKRL;
    std_msgs::Float32MultiArray realDataToKRL;
    std_msgs::Float32MultiArray realDataFromKRL;

    void PTP(const std::vector<double>& ptp,const std::vector<bool>& mask);
    void setTool(int tool_number);
    void setBase(int base_number);
    

    void setJointImpedanceControlMode();
    void setCartesianImpedanceControlMode();
    void setJointPositionControlMode();
    void setJointTorqueControlMode();

    bool setJointImpedanceControlModeROSService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& resp);
    bool setCartesianImpedanceControlModeROSService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& resp);
    bool setJointPositionControlModeROSService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& resp);
    bool setJointTorqueControlModeROSService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& resp);

    bool getCurrentControlModeROSService(std_srvs::TriggerRequest& req,std_srvs::TriggerResponse& resp);
    bool getCurrentControlMode();

    bool isJointPositionMode();
    bool isJointTorqueMode();
    bool isCartesianImpedanceMode();
    bool isJointImpedanceMode();

    RTT::OutputPort<lwr_fri::FriJointImpedance> port_JointImpedanceCommand;

    void resetJointImpedanceGains();
    void setStiffnessZero();

private:
    bool do_update;
    lwr_fri::FriJointImpedance cmd;
    bool is_joint_torque_control_mode;
    bool do_send_imp_cmd;
    bool is_initialized;
};
}
ORO_CREATE_COMPONENT(lwr::KRLTool)
#endif
