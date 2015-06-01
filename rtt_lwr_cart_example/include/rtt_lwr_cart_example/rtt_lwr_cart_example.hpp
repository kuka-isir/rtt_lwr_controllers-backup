// Copyright 2015 ISIR-CNRS
// Author: Antoine Hoarau

#ifndef __RTT_LWR_CART_EXAMPLE_HPP__
#define __RTT_LWR_CART_EXAMPLE_HPP__

#include "rtt_lwr_abstract/rtt_lwr_abstract.hpp"

#ifdef MARKERS
#include "rtt_lwr_cart_example/marker.hpp"
#endif

namespace lwr{
class RTTLWRCartExample : public RTTLWRAbstract{
public:
    RTTLWRCartExample(const std::string& name);
    virtual ~RTTLWRCartExample(){};
    void updateHook();
    bool configureHook();
    double ks_,k_lin,k_ang,d_lin,d_ang;
    int cnt_;
    geometry_msgs::PoseStamped cart_cmd_stamped;
    RTT::OutputPort<geometry_msgs::PoseStamped> port_PoseCommandStamped;
};
}
ORO_CREATE_COMPONENT(lwr::RTTLWRCartExample)
#endif
