// Copyright 2015 ISIR
// Author: Antoine Hoarau <hoarau.robotics@gmail.com>

#include "rtt_lwr_krl_tool/rtt_lwr_krl_tool.hpp"
namespace lwr{

KRLTool::KRLTool(const std::string& name): TaskContext(name)
{
    this->ports()->addPort("intDataToKRL_ros",port_intDataToKRL_ros).doc("");
    this->ports()->addPort("intDataFromKRL_ros",port_intDataFromKRL_ros).doc("");
    this->ports()->addPort("realDataToKRL_ros",port_realDataToKRL_ros).doc("");
    this->ports()->addPort("realDataFromKRL_ros",port_realDataFromKRL_ros).doc("");
    this->ports()->addPort("toKRL",port_toKRL).doc("");
    this->ports()->addPort("fromKRL",port_fromKRL).doc("");
    this->ports()->addPort("JointImpedanceCommand",port_JointImpedanceCommand).doc("");
    
    this->addOperation("setJointImpedanceMode",&KRLTool::setJointImpedanceMode,this,RTT::OwnThread);
    this->addOperation("setJointPositionMode",&KRLTool::setJointPositionMode,this,RTT::OwnThread);
    this->addOperation("resetJointImpedanceGains",&KRLTool::resetJointImpedanceGains,this,RTT::OwnThread);
    this->addOperation("setStiffnessZero",&KRLTool::setStiffnessZero,this,RTT::OwnThread);
}
void KRLTool::resetJointImpedanceGains()
{
    lwr_fri::FriJointImpedance cmd;
    for(unsigned int i=0;i<cmd.stiffness.size();++i){
        cmd.stiffness[i] = 1000.;
        cmd.damping[i] = 0.7;
    }
    port_JointImpedanceCommand.write(cmd);
}
void KRLTool::setStiffnessZero()
{
    lwr_fri::FriJointImpedance cmd;
    for(unsigned int i=0;i<cmd.damping.size();++i){
        cmd.stiffness[i] = 0.0;
        cmd.damping[i] = 0.0;
    }
    port_JointImpedanceCommand.write(cmd);
}

bool KRLTool::setJointImpedanceMode()
{
        int n=1000;
    while(n-- > 0 && port_fromKRL.read(fromKRL) != RTT::NewData)
        RTT::log(RTT::Debug) << "Waiting for new data s" << RTT::endlog();
    if(n==0)
        return false;
        for(unsigned i=0;i<FRI_USER_SIZE;++i)
            toKRL.intData[i] = fromKRL.intData[i];
    toKRL.intData[1] = 30;
    port_toKRL.write(toKRL);
    return isJointImpedanceMode();
}
bool KRLTool::isJointPositionMode()
{
    int n=1000;
    while(n-- > 0 && port_fromKRL.read(fromKRL) != RTT::NewData)
        ;//RTT::log(RTT::Debug) << "Waiting for new data i" << RTT::endlog();

    return fromKRL.intData[1] == 10;
}
bool KRLTool::isJointImpedanceMode()
{
    int n=1000;
    while(n-- > 0 && port_fromKRL.read(fromKRL) != RTT::NewData)
        RTT::log(RTT::Debug) << "Waiting for new data i" << RTT::endlog();

    return fromKRL.intData[1] == 30;
}
bool KRLTool::setJointPositionMode()
{
    int n=1000;
    while(n-- > 0 && port_fromKRL.read(fromKRL) != RTT::NewData)
        ;//RTT::log(RTT::Debug) << "Waiting for new data s" << RTT::endlog();
    if(n==0)
        return false;
    
    for(unsigned i=0;i<FRI_USER_SIZE;++i)
        toKRL.intData[i] = fromKRL.intData[i];
    
    toKRL.intData[1] = 10;
    port_toKRL.write(toKRL);
    return isJointPositionMode();
}
bool KRLTool::configureHook()
{
    for(unsigned i=0;i<FRI_USER_SIZE;++i){
        intDataToKRL.data.push_back(0);
        realDataToKRL.data.push_back(0.0);
        intDataFromKRL.data.push_back(0);
        realDataFromKRL.data.push_back(0.0);
    }  
    port_intDataToKRL_ros.createStream(rtt_roscomm::topic("~"+getName()+"/intDataToKRL"));
    port_realDataToKRL_ros.createStream(rtt_roscomm::topic("~"+getName()+"/realDataToKRL"));
    
    port_intDataFromKRL_ros.createStream(rtt_roscomm::topic("~"+getName()+"/intDataFromKRL"));
    port_realDataFromKRL_ros.createStream(rtt_roscomm::topic("~"+getName()+"/realDataFromKRL"));
    return true;
}

void KRLTool::updateHook()
{
    send_diag = false;
    
    if(port_fromKRL.read(fromKRL) != RTT::NewData)
        return;
    if(port_intDataToKRL_ros.read(intDataToKRL) == RTT::NewData)
    { 
        for(unsigned i=0;i<FRI_USER_SIZE && i<intDataToKRL.data.size();++i)
        {
            toKRL.intData[i] = fromKRL.intData[i];
            if(intDataToKRL.data[i] >= 0)
                toKRL.intData[i] = static_cast<fri_int32_t>(intDataToKRL.data[i]);
        }
        send_diag = true;
    }
    if(port_realDataToKRL_ros.read(realDataToKRL) == RTT::NewData)
    {  
        for(unsigned i=0;i<FRI_USER_SIZE && i<realDataToKRL.data.size();++i)
        {
            toKRL.realData[i]  = fromKRL.realData[i];
            if(realDataToKRL.data[i] >= 0)
                toKRL.realData[i] = static_cast<fri_float_t>(realDataToKRL.data[i]);
        }
        send_diag = true;
    }
    
    // To KRL
    
    if(send_diag)
    {
        port_toKRL.write(toKRL);
    }
    
    // ROS
    for(unsigned i=0;i<FRI_USER_SIZE && i<intDataFromKRL.data.size();++i)
    {
        intDataFromKRL.data[i] = fromKRL.intData[i];
    }
    for(unsigned i=0;i<FRI_USER_SIZE && i<realDataFromKRL.data.size();++i)
        realDataFromKRL.data[i] = fromKRL.realData[i];
    
    port_intDataFromKRL_ros.write(intDataFromKRL);
    port_realDataFromKRL_ros.write(realDataFromKRL);
        
}

}