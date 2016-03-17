#include "rtt_lwr_jt_controller/rtt_lwr_jt_controller.hpp"

namespace lwr{
using namespace RTT;
using namespace RTT::os;
JtATIController::JtATIController(const std::string& name):
use_kdl_gravity(false),
use_kdl(true),
add_damping(true),
compensate_coriolis(false),
use_ft_sensor(true),
RTTLWRAbstract(name)
{
    this->addPort("ft_data_in",port_ftdata).doc("");
    this->addProperty("use_kdl",use_kdl).doc("");
    this->addProperty("use_kdl_gravity",use_kdl_gravity).doc("");
    this->addProperty("add_damping",add_damping).doc("");
    this->addProperty("compensate_coriolis",compensate_coriolis).doc("");
    this->addProperty("use_ft_sensor",use_ft_sensor).doc("");
    this->addOperation("setDamping",&JtATIController::setDamping,this).doc("");
}

void JtATIController::setDamping(double d)
{
    if(d)
	kd.setConstant(d);
}

bool JtATIController::configureHook()
{
    if(!RTTLWRAbstract::init()) return false;
    this->setJointTorqueControlMode();

    J_kdl.resize(getNrOfJoints());

    mass_kdl.resize(getNrOfJoints());
    RTT::log(RTT::Warning) << "Last segment is : " <<
    kdl_chain.getSegment(kdl_chain.getNrOfSegments()-1).getName()<< RTT::endlog();

    kg.resize(getNrOfJoints());
    kg.setConstant(1.0);
    //HACK : kg[1] = 1.1;

    kd.resize(getNrOfJoints());
    kd.setConstant(10.0);

    return true;
}

void JtATIController::updateHook()
{
    log(RTT::Debug) << getName() << " UpdateHook() "<< TimeService::Instance()->getNSecs() << endlog();
    if(use_ft_sensor && port_ftdata.read(wrench_msg) != RTT::NewData)
        return;

    if(!updateState())
        {
            return;
        }

    jnt_trq_cmd.setZero();

    if(0)
    {
        getMassMatrix(mass);

        id_dyn_solver->JntToMass(jnt_pos_kdl,mass_kdl);

        RTT::log(RTT::Warning) << "mass : \n"<<mass<<RTT::endlog();
        RTT::log(RTT::Warning) << "mass_kdl : \n"<<mass_kdl.data<<RTT::endlog();
    }


    if(use_ft_sensor){
        if(use_kdl){
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
            RTT::log(RTT::Debug) << "Adding FT data    : \n"<<jnt_trq_cmd.transpose()<<RTT::endlog();

        }else{
            // Get Jacobian with KRC
            getJacobian(J_tip_base);
            getCartesianPosition(cart_pos);
            tf::poseMsgToKDL(cart_pos,tool_in_base_frame);
            tf::poseMsgToEigen(cart_pos,tool_in_base_frame_eigen);
            J_tip_base.changeBase(tool_in_base_frame.M);

            RTT::log(RTT::Debug) << "J : \n"<<J_tip_base.data<<RTT::endlog();


            // Get Jacobian with KDL
            jnt_to_jac_solver->JntToJac(jnt_pos_kdl,J_kdl,kdl_chain.getNrOfSegments());

            RTT::log(RTT::Debug) << "J_kdl : \n"<<J_kdl.data<<RTT::endlog();

            tf::wrenchMsgToEigen(wrench_msg.wrench,wrench);
            //tf::wrenchMsgToKDL(wrench_msg.wrench,wrench_kdl);
            wrench_kdl = tool_in_base_frame.M * wrench_kdl;
            ///tf::wrenchKDLToEigen(wrench_kdl,wrench);
            jnt_trq_cmd = J_tip_base.data.transpose() * wrench;

        }
    }

    if(use_kdl_gravity)
    {
        getGravityTorque(jnt_grav);
        id_dyn_solver->JntToGravity(jnt_pos_kdl,gravity_kdl);
        jnt_trq_cmd += kg.asDiagonal() * gravity_kdl.data - jnt_grav;
        RTT::log(RTT::Debug) << "Adding gravity    : \n"<<(kg.asDiagonal() * gravity_kdl.data - jnt_grav).transpose()<<RTT::endlog();
    }

    if(compensate_coriolis)
    {
        id_dyn_solver->JntToCoriolis(jnt_pos_kdl,jnt_vel_kdl,coriolis_kdl);
        jnt_trq_cmd -= coriolis_kdl.data.asDiagonal()*jnt_vel;
        RTT::log(RTT::Debug) << "Removing Coriolis    : \n"<<(coriolis_kdl.data.asDiagonal()*jnt_vel).transpose()<<RTT::endlog();
    }

    if(add_damping)
    {
        jnt_trq_cmd -= kd.asDiagonal() * jnt_vel;
        RTT::log(RTT::Debug) << "Adding damping    : \n"<<-(kd.asDiagonal() * jnt_vel).transpose()<<RTT::endlog();
    }

    RTT::log(RTT::Debug) << "jnt_trq_cmd    : \n"<<jnt_trq_cmd.transpose()<<RTT::endlog();
    usleep(2E6);
    sendJointTorque(jnt_trq_cmd);
}

}
