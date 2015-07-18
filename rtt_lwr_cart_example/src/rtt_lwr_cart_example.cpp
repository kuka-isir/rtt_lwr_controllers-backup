// Copyright 2015 ISIR
// Author: Antoine Hoarau <hoarau.robotics@gmail.com>

#include "rtt_lwr_cart_example/rtt_lwr_cart_example.hpp"
#ifdef MARKERS
using namespace markers_tests;
#endif

lwr::RTTLWRCartExample::RTTLWRCartExample(const std::string& name): 
RTTLWRAbstract(name),
cnt_(0),
ks_(0.03),
k_lin(1000.0),
k_ang(10.0),
d_lin(63.0),
d_ang(.1)
{
    this->addAttribute("ks",ks_);
    this->addAttribute("k_lin",k_lin);
    this->addAttribute("k_ang",k_ang);
    this->addAttribute("d_lin",d_lin);
    this->addAttribute("d_ang",d_ang);
    this->addPort("PoseCommandStamped",port_PoseCommandStamped).doc("");
}

bool lwr::RTTLWRCartExample::configureHook()
{
    bool configure = lwr::RTTLWRAbstract::configureHook();
    initializeCommand();
    setCartesianImpedanceControlMode();
            
    //initialise the command
    cart_pos_cmd.position.x = 0.43;
    cart_pos_cmd.position.y = -0.37;
    cart_pos_cmd.position.z = 0.42;
#ifdef MARKERS
    link_base_ = this->root_link;
    writing = false;
    marker_initialized = false;
    ros::NodeHandle n;
    tf::Vector3 position;
    position = tf::Vector3( cart_pos_cmd.position.x, cart_pos_cmd.position.y, cart_pos_cmd.position.z);
    ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);
    server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );
    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true );
    server->applyChanges();
#endif

         
        
    cart_cmd_stamped.header.frame_id = this->root_link;   
    
    port_PoseCommandStamped.createStream(rtt_roscomm::topic("~" + getName() +"/cart_cmd"));
    
    return configure;
}

void lwr::RTTLWRCartExample::updateHook()
{
            // Defaults in Kuka manual   
    cart_imp_cmd.stiffness.linear.x = k_lin;
    cart_imp_cmd.stiffness.linear.y = k_lin;
    cart_imp_cmd.stiffness.linear.z = k_lin;
    
    cart_imp_cmd.stiffness.angular.x = k_ang;
    cart_imp_cmd.stiffness.angular.y = k_ang;
    cart_imp_cmd.stiffness.angular.z = k_ang;
    
    cart_imp_cmd.damping.linear.x = d_lin; 
    cart_imp_cmd.damping.linear.y = d_lin;
    cart_imp_cmd.damping.linear.z = d_lin;
    
    cart_imp_cmd.damping.angular.x = d_ang; 
    cart_imp_cmd.damping.angular.y = d_ang;
    cart_imp_cmd.damping.angular.z = d_ang; 
    
    port_CartesianImpedanceCommand.write(cart_imp_cmd);
    
    
    
    if(isCommandMode() && isPowerOn())
    {
#ifdef MARKERS
      if(!writing && marker_initialized){
        cart_pos_cmd = marker_pose;
      }
#else
        if (0 <= cnt_ && cnt_ < 10/(double)getPeriod())
            cart_pos_cmd.position.x += ks_*(double)getPeriod();
        
        if(10/(double)getPeriod() <= cnt_ && cnt_ < 20/(double)getPeriod())
            cart_pos_cmd.position.z += ks_*(double)getPeriod();
        
        if(20/(double)getPeriod() <= cnt_ && cnt_ < 30/(double)getPeriod())
            cart_pos_cmd.position.z -= ks_*(double)getPeriod();
        
        if(30/(double)getPeriod() <= cnt_ && cnt_ < 40/(double)getPeriod())
            cart_pos_cmd.position.x -= ks_*(double)getPeriod();
        
        if(40/(double)getPeriod() <= cnt_)
            cnt_=0;

#endif
        cnt_++;
        
        cart_cmd_stamped.header.stamp = rtt_rosclock::host_now();
        cart_cmd_stamped.pose = cart_pos_cmd;
        sendCartesianPosition(cart_pos_cmd);
        port_PoseCommandStamped.write(cart_cmd_stamped);
    }
}