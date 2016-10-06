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
#include <rtt/Time.hpp>

#include <rtt_roscomm/rtt_rostopic.h>
#include <rtt_rosparam/rosparam.h>
#include <rtt_roscomm/rosservice.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>

#include <lwr_fri/FriJointImpedance.h>
#include <kuka_lwr_fri/friComm.h>
#include <rtt_lwr_krl_tool/fri_user_data_description.h>

#include <krl_msgs/PTPAction.h>
#include <krl_msgs/LINAction.h>
#include <krl_msgs/SetMaxVelPercent.h>
#include <krl_msgs/SetTool.h>
#include <krl_msgs/SetBase.h>
#include <krl_msgs/SetToolBase.h>

#include <rtt_actionlib/rtt_action_server.h>

template<typename T> static bool getBit(const T& in, unsigned int bit_number)
{
    return ((in >> bit_number) & 1);
}

template<typename T> static void setBit(T& in, unsigned int bit_number,bool status)
{
    in ^= (-status ^ in) & (1 << bit_number);
    return;
}

namespace lwr{
static const int ROS_MASK_NO_UPDATE = -9999;

class KRLTool : public RTT::TaskContext{

private:
    // PTP + PTP_REL action
    typedef actionlib::ServerGoalHandle<krl_msgs::PTPAction> PTPGoalHandle;
    rtt_actionlib::RTTActionServer<krl_msgs::PTPAction> ptp_action_server_;
    rtt_actionlib::RTTActionServer<krl_msgs::PTPAction> ptp_rel_action_server_;

    // LIN + LIN_REL action
    typedef actionlib::ServerGoalHandle<krl_msgs::LINAction> LINGoalHandle;
    rtt_actionlib::RTTActionServer<krl_msgs::LINAction> lin_action_server_;
    rtt_actionlib::RTTActionServer<krl_msgs::LINAction> lin_rel_action_server_;

    PTPGoalHandle ptp_current_gh;
    krl_msgs::PTPResult ptp_result;
    LINGoalHandle lin_current_gh;
    krl_msgs::LINResult lin_result;
    // krl_msgs::LINFeedback lin_feedback;
    // krl_msgs::PTPFeedback ptp_feedback;
    // std::vector<float> startPTP,startLIN;

public:
    KRLTool(const std::string& name);
    virtual ~KRLTool(){};
    void updateHook();
    bool configureHook();
    bool startHook();
    void stopHook();
    bool FRIOpen(int period_ms = 1 /*ms*/);
    bool FRIStart();
    bool FRIStop();
    bool FRIClose();

// Actionlib Callbacks
protected:
    void PTPgoalCallback(PTPGoalHandle gh);
    void PTPcancelCallback(PTPGoalHandle gh);
    void LINgoalCallback(LINGoalHandle gh);
    void LINcancelCallback(LINGoalHandle gh);

protected:
    RTT::InputPort<tFriKrlData> port_fromKRL;
    RTT::OutputPort<tFriKrlData> port_toKRL;
    tFriKrlData toKRL;
    tFriKrlData fromKRL;
    geometry_msgs::WrenchStamped ft_sensor_wrench;
    RTT::InputPort<geometry_msgs::WrenchStamped> port_ft_sensor;
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

    void printBool();
    void printInt();
    void printReal();
    void printAll();
    bool sendFRICommand(int cmd,bool wait_until_done = true);

private: void PointToPoint(
    const std::vector<double>& ptp,
    const std::vector<bool>& mask,
    const geometry_msgs::Vector3& XYZ,
    const geometry_msgs::Vector3& RPY,
    const geometry_msgs::Vector3& XYZ_mask,
    const geometry_msgs::Vector3& RPY_mask,
    bool use_radians,
    int ptp_input_type,
    bool use_rel,
    double vel_percent);

private: void Linear(
    const geometry_msgs::Vector3& XYZ_meters,
    const geometry_msgs::Vector3& RPY_rad,
    const geometry_msgs::Vector3& XYZ_mask,
    const geometry_msgs::Vector3& RPY_mask,
    bool use_rel,
    double vel_percent,
    bool in_tool_frame = false);

protected:
    void addNoAckNeededVar(int special_case);
    bool isNoAckNeededVar(int test_case);
    void resetData();
    void cancelMotion();
    void cancelGoals();
    void setTool(int tool_number);
    void setBase(int base_number);
    void sendSTOP2();
    void unsetSTOP2();
    void setVELPercent(float vel_percent);
    bool sendSTOP2_srv(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
    bool unsetSTOP2_srv(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);

    bool setMaxVelPercent_srv(krl_msgs::SetMaxVelPercentRequest& req,krl_msgs::SetMaxVelPercentResponse& resp);

    bool setTool_srv(krl_msgs::SetToolRequest& req,krl_msgs::SetToolResponse& resp);
    bool setBase_srv(krl_msgs::SetBaseRequest& req,krl_msgs::SetBaseResponse& resp);
    bool setToolBase_srv(krl_msgs::SetToolBaseRequest& req,krl_msgs::SetToolBaseResponse& resp);

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
    void verifySpecialCases();
    void resetBoolToKRL();
    bool hasKRLReset();
    lwr_fri::FriJointImpedance cmd;
    bool is_joint_torque_control_mode;
    bool do_send_imp_cmd;
    bool is_initialized;
    std::vector<int> bypass_ack_idx;
};
}
ORO_CREATE_COMPONENT(lwr::KRLTool)
#endif
