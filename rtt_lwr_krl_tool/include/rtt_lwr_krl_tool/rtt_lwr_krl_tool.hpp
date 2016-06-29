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

namespace lwr{
    
    static const size_t CONTROL_MODE    = 1;
    static const size_t FRI_STATE       = 0;
    static const int NO_UPDATE = -9999;
    
class KRLTool : public RTT::TaskContext{
public:
    KRLTool(const std::string& name);
    virtual ~KRLTool(){};
    void updateHook();
    bool configureHook();
protected:
    RTT::OutputPort<lwr_fri::FriJointImpedance> port_JointImpedanceCommand;
    RTT::InputPort<tFriKrlData> port_fromKRL;
    RTT::OutputPort<tFriKrlData> port_toKRL;
    RTT::InputPort<std_msgs::Int32MultiArray> port_intDataToKRL_ros;
    RTT::OutputPort<std_msgs::Int32MultiArray> port_intDataFromKRL_ros;
    RTT::InputPort<std_msgs::Float32MultiArray> port_realDataToKRL_ros;
    RTT::OutputPort<std_msgs::Float32MultiArray> port_realDataFromKRL_ros;
    
    tFriKrlData toKRL;
    tFriKrlData fromKRL;

    std_msgs::Int32MultiArray intDataToKRL;
    std_msgs::Int32MultiArray intDataFromKRL;

    std_msgs::Float32MultiArray realDataToKRL;
    std_msgs::Float32MultiArray realDataFromKRL;
    
    void setJointImpedanceControlMode();
    void setCartesianImpedanceControlMode();
    void setJointPositionControlMode();
    void setJointTorqueControlMode();
    
    bool setJointImpedanceControlModeROSService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& resp);
    bool setCartesianImpedanceControlModeROSService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& resp);
    bool setJointPositionControlModeROSService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& resp);
    bool setJointTorqueControlModeROSService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& resp);
    
    bool getCurrentControlModeROSService(std_srvs::TriggerRequest& req,std_srvs::TriggerResponse& resp);
        
    bool isJointPositionMode();
    bool isJointTorqueMode();
    bool isCartesianImpedanceMode();
    bool isJointImpedanceMode();
    
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
