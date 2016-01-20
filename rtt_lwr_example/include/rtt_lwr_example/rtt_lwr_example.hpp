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
    double freq_;
    int cnt_;
    bool initialized_;
    void setGains(unsigned int i,double p,double d);
    bool setLinkEnabled(unsigned int i,bool val);
    double amplitude_;
    std::vector<bool> link_enabled_;
    Eigen::VectorXd jnt_pos_init;

};
}
ORO_CREATE_COMPONENT(lwr::RTTLWRExample)
#endif
