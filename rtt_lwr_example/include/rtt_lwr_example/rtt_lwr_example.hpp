// Copyright 2015 ISIR-CNRS
// Author: Antoine Hoarau

#ifndef __RTT_LWR_EXAMPLE_HPP__
#define __RTT_LWR_EXAMPLE_HPP__

#include "rtt_lwr_abstract/rtt_lwr_abstract.hpp"

namespace lwr{
class RTTLWRExample : public RTTLWRAbstract{
public:
    RTTLWRExample(const std::string& name);
    virtual ~RTTLWRExample(){};
    void updateHook();
    bool configureHook();
    double ks_;
    int cnt_;
    bool initialized_;
    void setAmplitude(double amplitude);
    std::vector<double> kp,kd;
    void setGains(double p,double d);
    bool torque_only;
    double amplitude_;
    
};
}
ORO_CREATE_COMPONENT(lwr::RTTLWRExample)
#endif
