// Copyright 2015 ISIR
// Author: Antoine Hoarau <hoarau.robotics@gmail.com>

#include "rtt_lwr_example/rtt_lwr_example.hpp"


lwr::RTTLWRExample::RTTLWRExample(const std::string& name):
RTTLWRAbstract(name),
cnt_(0),
freq_(1.0),
initialized_(false),
link_enabled_(LBR_MNJ,true),
jnt_pos_init(LBR_MNJ),
amplitude_(10.0*M_PI/180.)
{
    this->addAttribute("frequence",freq_);
    this->addAttribute("amplitude",amplitude_);
    this->addAttribute("initialized",initialized_);
    this->addOperation("setGains",&lwr::RTTLWRExample::setGains,this,RTT::OwnThread);
    this->addOperation("setLinkEnabled",&lwr::RTTLWRExample::setLinkEnabled,this,RTT::OwnThread);
}
bool lwr::RTTLWRExample::setLinkEnabled(unsigned int i,bool val)
{
    if(i<LBR_MNJ)
    {
        link_enabled_[i] = val;
    }
    return link_enabled_[i];
}
bool lwr::RTTLWRExample::configureHook()
{
    bool configure = lwr::RTTLWRAbstract::init();
    //initializeCommand();
    setJointImpedanceControlMode();
    //setJointTorqueControlMode();
    return configure;
}
void lwr::RTTLWRExample::setGains(unsigned int j_idx,double p, double d)
{
    for(int i=0;i<LBR_MNJ;i++){
        jnt_imp_cmd.damping[i] = -1;
        jnt_imp_cmd.stiffness[i] = -1;
    }
    if(j_idx<LBR_MNJ){
        jnt_imp_cmd.damping[j_idx] = d;
        jnt_imp_cmd.stiffness[j_idx] = p;
    }
    sendJointImpedance(jnt_imp_cmd);
}

void lwr::RTTLWRExample::updateHook()
{

    if(isCommandMode() && isPowerOn()){

            getJointVelocity(jnt_vel);
            getJointPosition(jnt_pos);
            if(!initialized_)
            {
                for(int i=0;i<LBR_MNJ;i++)
                    jnt_pos_init[i] = jnt_pos[i];
                initialized_ = true;
            }
            for(int i=0;i<LBR_MNJ;i++)
                if(!link_enabled_[i])
                    jnt_pos_init[i] = jnt_pos[i];

            for(unsigned i=0;i<LBR_MNJ;++i)
            {
                if(link_enabled_[i])
                    this->jnt_pos_cmd[i] = jnt_pos_init[i] + amplitude_*sin((double)cnt_*(double)getPeriod()*freq_);//*double(i+1));
                else
                    this->jnt_pos_cmd[i] = jnt_pos_init[i];
            }
            //RTT::log(RTT::Info) << "jnt_trq_cmd="<<jnt_pos_cmd.transpose()<<RTT::endlog();
            //RTT::log(RTT::Info) << "jnt_pos_cur="<<jnt_pos.transpose()<<RTT::endlog();

            sendJointPosition(jnt_pos_cmd);
            cnt_++;
        }

}
