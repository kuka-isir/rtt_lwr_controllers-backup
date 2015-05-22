// Copyright 2015 ISIR
// Author: Antoine Hoarau <hoarau.robotics@gmail.com>

#include "rtt_lwr_example/rtt_lwr_example.hpp"


lwr::RTTLWRExample::RTTLWRExample(const std::string& name): RTTLWRAbstract(name)
{
    RTT::log(RTT::Info) << "Example created" << RTT::endlog();
}

bool lwr::RTTLWRExample::configureHook()
{
    bool configure = lwr::RTTLWRAbstract::configureHook();

    this->setJointPositionControlMode();

    return configure;
}

void lwr::RTTLWRExample::updateHook()
{
    //RTT::log(RTT::Info) << "updateHook() loop" << RTT::endlog();
}