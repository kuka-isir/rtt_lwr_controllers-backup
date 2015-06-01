// Copyright 2015 ISIR
// Author: Antoine Hoarau <hoarau.robotics@gmail.com>

#include "rtt_lwr_example/rtt_lwr_example.hpp"


lwr::RTTLWRExample::RTTLWRExample(const std::string& name): 
RTTLWRAbstract(name),
cnt_(0),
ks_(0.1),
initialized_(false),
kp(LBR_MNJ,0.1),
kd(LBR_MNJ,0.001),
torque_only(false)
{
    this->addAttribute("ks",ks_);
    this->addProperty("torque_only",torque_only);
    this->addAttribute("initialized",initialized_);
    this->addOperation("setAmplitude",&lwr::RTTLWRExample::setAmplitude,this,RTT::OwnThread);
    this->addOperation("setGains",&lwr::RTTLWRExample::setGains,this,RTT::OwnThread);
}

bool lwr::RTTLWRExample::configureHook()
{
    bool configure = lwr::RTTLWRAbstract::configureHook();
    initializeCommand();
    setJointImpedanceControlMode();
    return configure;
}
void lwr::RTTLWRExample::setGains(double p, double d)
{
    for(unsigned i=0;i<LBR_MNJ;++i){
        kp[i] = p;
        kd[i] = d;
    }
}

void lwr::RTTLWRExample::setAmplitude(double amplitude)
{
    ks_ = amplitude;
}

void lwr::RTTLWRExample::updateHook()
{
    
    if(isCommandMode()){
        if((!initialized_) && getJointPosition(jnt_pos))
            {
                RTT::log(RTT::Info) << " Joint Position Initialized ! "<<jnt_pos.transpose()<<RTT::endlog();     
                initialized_ = true;
            }
            
        if(initialized_ && isPowerOn())
        {
            if(torque_only){
                getJointVelocity(jnt_vel);
                getJointPosition(jnt_pos);
                
                    for(unsigned k=0;k<LBR_MNJ;++k)
                    {
                        jnt_imp_cmd.damping[k] = 0.7;
                        jnt_imp_cmd.stiffness[k] = 0;
                    }
            }else{
                    for(unsigned i=0;i<LBR_MNJ;++i)
                        {
                            jnt_imp_cmd.damping[i] = 0.7;
                            jnt_imp_cmd.stiffness[i] = 1000.0;
                        }
            }
            for(unsigned i=0;i<n_joints;++i)
            {
                this->jnt_pos_cmd[i] = /*jnt_pos[i] +*/ 30*3.14/180.0*sin(cnt_*getPeriod()*ks_);//*double(i+1));
                
                if(torque_only){

                    this->jnt_trq_cmd[i] = kp[i]*(jnt_pos_cmd[i] - jnt_pos[i]) - kd[i]*jnt_vel[i];
                }
            }
            //RTT::log(RTT::Info) << "jnt_trq_cmd="<<jnt_trq_cmd.transpose()<<RTT::endlog();
            //RTT::log(RTT::Info) << "jnt_pos_cur="<<jnt_pos.transpose()<<RTT::endlog();
            
            if(torque_only)
                sendJointTorque(jnt_trq_cmd);
            else
               sendJointPosition(jnt_pos_cmd); 
            
            sendJointImpedance(jnt_imp_cmd);
            cnt_++;
        
        }else{

        }
    }else{
        //RTT::log(RTT::Debug) << "isCommandMode()="<<isCommandMode()<<" isPowerOn()="<<isPowerOn()<<RTT::endlog();
    }
}