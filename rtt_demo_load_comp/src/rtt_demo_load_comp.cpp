#include "rtt_demo_load_comp/rtt_demo_load_comp.hpp"
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>

using namespace RTT;
using namespace RTT::os;

LoadCompDemo::LoadCompDemo(const std::string& name):
TaskContext(name),
add_damping(true),
compensate_coriolis(false),
ft_sensor_link("ati_link")
{
    this->addPort("ft_data_in",port_ftdata).doc("");
    this->addProperty("ft_sensor_link",ft_sensor_link).doc("");
    this->addProperty("damping",damping).doc("");
    this->addProperty("add_damping",add_damping).doc("");
    this->addProperty("compensate_coriolis",compensate_coriolis).doc("");

    this->addPort("JointPosition",port_joint_position_in).doc("Current joint positions");
    this->addPort("JointVelocity",port_joint_velocity_in).doc("Current joint velocities");

    this->addPort("JointTorqueCommand",port_joint_torque_cmd_out).doc("Command joint torques");

}

bool LoadCompDemo::configureHook()
{
    if(!arm.init()) return false;

    damping.resize(arm.getNrOfJoints());
    kg.resize(arm.getNrOfJoints());
    zero_kdl.resize(arm.getNrOfJoints());

    damping.setConstant(1.0);
    kg.setConstant(1.0);

    jnt_pos_in.setZero(arm.getNrOfJoints());
    jnt_vel_in.setZero(arm.getNrOfJoints());

    jnt_trq_cmd_out.setZero(arm.getNrOfJoints());

    port_joint_torque_cmd_out.setDataSample(jnt_trq_cmd_out);

    rtt_ros_kdl_tools::getAllPropertiesFromROSParam(this);


    return true;
}

void LoadCompDemo::updateHook()
{
    if(port_joint_position_in.read(jnt_pos_in) == NoData) return;
    port_joint_velocity_in.read(jnt_vel_in);

    if(port_ftdata.read(ft_msg) == NoData)
    {
       log(Error) << "No FT Data received" << endlog();
       this->error();
    }

    arm.setState(jnt_pos_in,jnt_vel_in);
    arm.updateModel();

    KDL::Wrench wrench_kdl;
    tf::wrenchMsgToKDL(ft_msg.wrench,wrench_kdl);

    arm.setExternalMeasuredWrench(wrench_kdl,arm.getSegmentIndex(ft_sensor_link));

    arm.computeExternalWrenchTorque(jnt_pos_in,true);

    KDL::JntArray& ext_t = arm.getExternalWrenchTorque();

    log(Debug) << "External torque : "<<ext_t.data.transpose()<<endlog();

    jnt_trq_cmd_out = ext_t.data;

    jnt_trq_cmd_out -= damping.asDiagonal() * jnt_vel_in * (add_damping ? 1 : 0);

    port_joint_torque_cmd_out.write(jnt_trq_cmd_out);
}
