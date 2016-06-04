// Copyright 2015 ISIR
// Author: Antoine Hoarau <hoarau.robotics@gmail.com>

#include "rtt_lwr_krl_tool/rtt_lwr_krl_tool.hpp"
namespace lwr{
using namespace RTT;

KRLTool::KRLTool(const std::string& name): 
TaskContext(name),
do_update(false),
do_send_imp_cmd(false),
is_joint_torque_control_mode(false)
{
    this->ports()->addPort("intDataToKRL_ros",port_intDataToKRL_ros).doc("");
    this->ports()->addPort("intDataFromKRL_ros",port_intDataFromKRL_ros).doc("");
    this->ports()->addPort("realDataToKRL_ros",port_realDataToKRL_ros).doc("");
    this->ports()->addPort("realDataFromKRL_ros",port_realDataFromKRL_ros).doc("");
    this->ports()->addPort("toKRL",port_toKRL).doc("Struct defined in friComm.h to send to the KRL Program");
    this->ports()->addEventPort("fromKRL",port_fromKRL).doc("Struct defined in friComm.h to read from KRL Program");
    this->ports()->addPort("JointImpedanceCommand",port_JointImpedanceCommand).doc("");

    this->addOperation("setJointImpedanceControlMode",&KRLTool::setJointImpedanceControlMode,this);
    this->addOperation("setJointTorqueControlMode",&KRLTool::setJointTorqueControlMode,this);
    this->addOperation("setJointPositionControlMode",&KRLTool::setJointPositionControlMode,this);
    this->addOperation("setCartesianImpedanceControlMode",&KRLTool::setCartesianImpedanceControlMode,this);

    this->addOperation("setJointImpedanceControlModeROSService",&KRLTool::setJointImpedanceControlModeROSService,this);
    this->addOperation("setJointTorqueControlModeROSService",&KRLTool::setJointTorqueControlModeROSService,this);
    this->addOperation("setJointPositionControlModeROSService",&KRLTool::setJointPositionControlModeROSService,this);
    this->addOperation("setCartesianImpedanceControlModeROSService",&KRLTool::setCartesianImpedanceControlModeROSService,this);

    this->addOperation("getCurrentControlModeROSService",&KRLTool::getCurrentControlModeROSService,this);
    
    this->addOperation("resetJointImpedanceGains",&KRLTool::resetJointImpedanceGains,this);
    this->addOperation("setStiffnessZero",&KRLTool::setStiffnessZero,this);
    
    for(int i=0;i<FRI_USER_SIZE;i++)
    {
        fromKRL.boolData = toKRL.boolData = 0;
        fromKRL.intData[i] = toKRL.intData[i] =  0;
        fromKRL.realData[i] = toKRL.realData[i] = 0.0;
    }
}

void KRLTool::resetJointImpedanceGains()
{
    for(unsigned int i=0;i<cmd.stiffness.size();++i){
        cmd.stiffness[i] = 1000.;
        cmd.damping[i] = 0.7;
    }
    do_send_imp_cmd = true;
}
void KRLTool::setStiffnessZero()
{
    for(unsigned int i=0;i<cmd.damping.size();++i){
        cmd.stiffness[i] = 0.0;
        cmd.damping[i] = 0.0;
    }
    do_send_imp_cmd = true;
    is_joint_torque_control_mode = true;
}

bool KRLTool::isJointPositionMode()
{
    return fromKRL.intData[CONTROL_MODE] == static_cast<int>(FRI_CTRL_POSITION)*10;
}
bool KRLTool::isJointTorqueMode()
{
    return is_joint_torque_control_mode;
}
bool KRLTool::isJointImpedanceMode()
{
    return fromKRL.intData[CONTROL_MODE] == static_cast<int>(FRI_CTRL_JNT_IMP)*10;
}

bool KRLTool::isCartesianImpedanceMode()
{
    return fromKRL.intData[CONTROL_MODE] == static_cast<int>(FRI_CTRL_CART_IMP)*10;
}

void KRLTool::setJointPositionControlMode()
{
    toKRL.intData[CONTROL_MODE] = static_cast<int>(FRI_CTRL_POSITION)*10;
    do_update = true;
}

void KRLTool::setJointImpedanceControlMode()
{
    toKRL.intData[CONTROL_MODE] = static_cast<int>(FRI_CTRL_JNT_IMP)*10;
    do_update = true;
}

void KRLTool::setCartesianImpedanceControlMode()
{
    toKRL.intData[CONTROL_MODE] = static_cast<int>(FRI_CTRL_CART_IMP)*10;
    do_update = true;
}

void KRLTool::setJointTorqueControlMode()
{
    setJointImpedanceControlMode();
    setStiffnessZero();
    do_update = true;
}

bool KRLTool::configureHook()
{
    for(unsigned i=0;i<FRI_USER_SIZE;++i){
        intDataToKRL.data.push_back(0);
        realDataToKRL.data.push_back(0.0);
        intDataFromKRL.data.push_back(0);
        realDataFromKRL.data.push_back(0.0);
    }
    port_intDataToKRL_ros.createStream(rtt_roscomm::topic(getName()+"/intDataToKRL"));
    port_realDataToKRL_ros.createStream(rtt_roscomm::topic(getName()+"/realDataToKRL"));

    port_intDataFromKRL_ros.createStream(rtt_roscomm::topic(getName()+"/intDataFromKRL"));
    port_realDataFromKRL_ros.createStream(rtt_roscomm::topic(getName()+"/realDataFromKRL"));
    return true;
}
bool KRLTool::getCurrentControlModeROSService(std_srvs::TriggerRequest& req,std_srvs::TriggerResponse& resp)
{
    resp.success = true;
    switch(fromKRL.intData[CONTROL_MODE])
    {
        case static_cast<int>(FRI_CTRL_POSITION)*10:
            resp.message = "Joint Position Mode - FRI_CTRL_POSITION";
            break;
        case static_cast<int>(FRI_CTRL_JNT_IMP)*10:
            resp.message = "Joint Impedance Mode - FRI_CTRL_JNT_IMP";
            break;
        case static_cast<int>(FRI_CTRL_CART_IMP)*10:
            resp.message = "Cartesian Impedance Mode -  FRI_CTRL_CART_IMP";
            break;
        case static_cast<int>(FRI_CTRL_OTHER)*10:
            resp.message = "FRI_CTRL_OTHER";
            break;
        default:
            resp.success = false;
            resp.message = "Error";    
    }
    log(Info) << "KRLTool::getCurrentControlModeROSService" << endlog();
}

bool KRLTool::setJointImpedanceControlModeROSService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& resp)
{
    log(Info) << "KRLTool::setJointImpedanceControlModeROSService" << endlog();
    setJointImpedanceControlMode();
}
bool KRLTool::setJointPositionControlModeROSService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& resp)
{
    log(Info) << "KRLTool::setJointPositionControlModeROSService" << endlog();
    setJointPositionControlMode();
}
bool KRLTool::setJointTorqueControlModeROSService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& resp)
{
    log(Info) << "KRLTool::setJointTorqueControlModeROSService" << endlog();
    setJointTorqueControlMode();
}
bool KRLTool::setCartesianImpedanceControlModeROSService(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
    log(Info) << "KRLTool::setCartesianImpedanceControlModeROSService" << endlog();
    setCartesianImpedanceControlMode();
}

void KRLTool::updateHook()
{
    if(port_fromKRL.read(fromKRL) != NewData)
        return;
    // NOTE : At this point we have a New KRL info
    
    // To ROS for plotting
    for(unsigned i=0;i<FRI_USER_SIZE && i<intDataFromKRL.data.size();++i)
        intDataFromKRL.data[i] = fromKRL.intData[i];
    for(unsigned i=0;i<FRI_USER_SIZE && i<realDataFromKRL.data.size();++i)
        realDataFromKRL.data[i] = fromKRL.realData[i];

    port_intDataFromKRL_ros.write(intDataFromKRL);
    port_realDataFromKRL_ros.write(realDataFromKRL);

    // Incoming ROS Int message
    if(port_intDataToKRL_ros.read(intDataToKRL) == NewData)
    {
        for(unsigned i=0;i<FRI_USER_SIZE && i<intDataToKRL.data.size();++i)
            if(intDataToKRL.data[i] != lwr::NO_UPDATE)
                toKRL.intData[i] = static_cast<fri_int32_t>(intDataToKRL.data[i]);
        do_update = true;
    }
    
    // Incoming ROS Float message
    if(port_realDataToKRL_ros.read(realDataToKRL) == NewData)
    {
        for(unsigned i=0;i<FRI_USER_SIZE && i<realDataToKRL.data.size();++i)
            if(realDataToKRL.data[i] != lwr::NO_UPDATE)
                toKRL.realData[i] = static_cast<fri_float_t>(realDataToKRL.data[i]);
        do_update = true;
    }

    // To KRL

    if(do_update){
        port_toKRL.write(toKRL);
        do_update = false;
    }
    // Joint Impedance Commands
    if(do_send_imp_cmd){
        port_JointImpedanceCommand.write(cmd);
        do_send_imp_cmd = false;
    }

}

}
