// Copyright 2015 ISIR
// Author: Antoine Hoarau <hoarau.robotics@gmail.com>

#include "rtt_lwr_example/rtt_lwr_example.hpp"


lwr::RTTLWRExample::RTTLWRExample(const std::string& name): RTTLWRAbstract(name),cnt_(0),ks_(1)
{
    this->addAttribute("ks",ks_);
}

bool lwr::RTTLWRExample::configureHook()
{
    bool configure = lwr::RTTLWRAbstract::configureHook();

    setJointPositionControlMode();
    return configure;
}

void lwr::RTTLWRExample::updateHook()
{
    cnt_++;
    for(unsigned i=0;i<n_joints_;++i)
    {
        if(i>=0)
            this->jnt_trq_cmd[i] = 60*3.14/180.0*sin(cnt_*getPeriod()*ks_);//*double(i+1));
        else
            this->jnt_trq_cmd[i] = 0.0;
    }
    sendJointPosition(jnt_trq_cmd);
}