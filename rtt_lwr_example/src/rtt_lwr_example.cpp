// Copyright 2015 ISIR
// Author: Antoine Hoarau <hoarau.robotics@gmail.com>

#include "rtt_lwr_example/rtt_lwr_example.hpp"


lwr::RTTLWRExample::RTTLWRExample(const std::string& name): 
RTTLWRAbstract(name),
cnt_(0),
ks_(1.0),
initialized_(false),
kp(LBR_MNJ,0.1),
kd(LBR_MNJ,0.001),
amplitude_(10.0*M_PI/180.),
torque_only(false)
{
    this->addAttribute("ks",ks_);
    this->addAttribute("amplitude",amplitude_);
    this->addProperty("torque_only",torque_only);
    this->addAttribute("initialized",initialized_);
    this->addOperation("setAmplitude",&lwr::RTTLWRExample::setAmplitude,this,RTT::OwnThread);
    this->addOperation("setGains",&lwr::RTTLWRExample::setGains,this,RTT::OwnThread);
}

bool lwr::RTTLWRExample::configureHook()
{
    bool configure = lwr::RTTLWRAbstract::init();
    //initializeCommand();
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
    amplitude_ = amplitude;
}

void lwr::RTTLWRExample::updateHook()
{
    
    if(isCommandMode() && isPowerOn()){
        if((!initialized_) && getJointPosition(jnt_pos))
            {
                RTT::log(RTT::Info) << " Joint Position Initialized ! "<<jnt_pos.transpose()<<RTT::endlog();     
                initialized_ = true;
            }
            
        if(initialized_)
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
            for(unsigned i=0;i<LBR_MNJ;++i)
            {
                this->jnt_pos_cmd[i] = jnt_pos[i] + amplitude_*sin((double)cnt_*(double)getPeriod()*ks_);//*double(i+1));
                
                if(torque_only){

                    this->jnt_trq_cmd[i] = kp[i]*(jnt_pos_cmd[i] - jnt_pos[i]) - kd[i]*jnt_vel[i];
                }
            }
            //RTT::log(RTT::Info) << "jnt_trq_cmd="<<jnt_pos_cmd.transpose()<<RTT::endlog();
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
    //this->trigger();
}