#include "rtt_lwr_jt_controller/rtt_lwr_jt_controller.hpp"

namespace lwr{

JtATIController::JtATIController(const std::string& name):RTTLWRAbstract(name)
{
    this->addPort("FTData",port_ftdata).doc("");
}

bool JtATIController::configureHook()
{
    bool configure = RTTLWRAbstract::configureHook();
    setJointImpedanceControlMode();
    std::fill(jnt_imp_cmd.stiffness.begin(),jnt_imp_cmd.stiffness.end(),0.0);
    sendJointImpedance(jnt_imp_cmd);
    J_kdl.resize(getNJoints());
    jnt_acc_kdl.resize(getNJoints());
    jnt_acc_kdl.data.setZero();
    //tf.waitForTransform(root_link,"/ati_link",ros::Time(0),ros::Duration(5));
    //tf::StampedTransform T_ati_lwr;
    //tf.lookupTransform(tip_link,"/ati_link",ros::Time(0),T_ati_lwr);
    
    return configure;
}

void JtATIController::updateHook()
{
    if(port_ftdata.read(wrench_msg) != RTT::NewData)
        return;
    
    getJacobian(J);
    updateState();
    getCartesianPosition(cart_pos);
    
    fk_vel_solver->JntToCart(jnt_pos_vel_kdl,tool_in_base_framevel);

    if(1){
        // remove B(q,qdot)
        jnt_vel_kdl.data.setZero();
        // Get the Wrench in the last segment's frame
        tf::wrenchMsgToKDL(wrench_msg.wrench,wrench_kdl);
        f_ext_kdl.back() = wrench_kdl;
        // Get The full dynamics with external forces
        id_rne_solver->CartToJnt(jnt_pos_kdl,jnt_vel_kdl,jnt_acc_kdl,f_ext_kdl,jnt_trq_kdl);
        // Get Joint Gravity torque
        id_dyn_solver->JntToGravity(jnt_pos_kdl,gravity_kdl);
        // remove G(q)
        jnt_trq_cmd = jnt_trq_kdl.data - gravity_kdl.data;
    
    }else{
        tf::poseMsgToKDL(cart_pos,tool_in_base_frame);
        J.changeBase(tool_in_base_frame.M);
        
        //RTT::log(RTT::Debug) << "J : \n"<<J.data<<RTT::endlog();
        
        jnt_to_jac_solver->JntToJac(jnt_pos_kdl,J_kdl);
        
        //RTT::log(RTT::Debug) << "J_kdl : \n"<<J_kdl.data<<RTT::endlog();
        
        
        getGravityTorque(jnt_grav);
        
        //RTT::log(RTT::Debug) << "gravity_kdl : \n"<<gravity_kdl.data.transpose()<<RTT::endlog();
        //RTT::log(RTT::Debug) << "jnt_grav    : \n"<<jnt_grav.transpose()<<RTT::endlog();
        
        tf::wrenchMsgToEigen(wrench_msg.wrench,wrench);
        jnt_trq_cmd = -J.data.transpose() * wrench;
    
    }
    
    RTT::log(RTT::Debug) << "jnt_trq_cmd    : \n"<<jnt_trq_cmd.transpose()<<RTT::endlog();
    
    //jnt_trq_cmd = gravity_kdl.data - jnt_grav;
    
    sendJointTorque(jnt_trq_cmd);
}

}
