// Copyright 2015 ISIR
// Author: Antoine Hoarau <hoarau.robotics@gmail.com>

#include "rtt_lwr_krl_tool/rtt_lwr_krl_tool.hpp"

namespace lwr{
using namespace RTT;
using namespace krl;

KRLTool::KRLTool(const std::string& name):
TaskContext(name),
do_update(false),
do_send_imp_cmd(false),
is_joint_torque_control_mode(false)
{
    this->ports()->addPort("intDataToKRL_ros",port_intDataToKRL_ros).doc("");
    this->ports()->addPort("intDataFromKRL_ros",port_intDataFromKRL_ros).doc("");
    this->ports()->addPort("realDataToKRL_ros",port_realDataToKRL_ros).doc("");
    this->ports()->addPort("realDataFromKRL_ros",port_realDataFromKRL_ros).doc("");
    this->ports()->addPort("boolDataFromKRL_ros",port_boolDataFromKRL_ros).doc("");
    this->ports()->addPort("toKRL",port_toKRL).doc("Struct defined in friComm.h to send to the KRL Program");
    this->ports()->addEventPort("fromKRL",port_fromKRL).doc("Struct defined in friComm.h to read from KRL Program");
    this->ports()->addPort("JointImpedanceCommand",port_JointImpedanceCommand).doc("");

    this->addOperation("setJointImpedanceControlMode",&KRLTool::setJointImpedanceControlMode,this);
    this->addOperation("setJointTorqueControlMode",&KRLTool::setJointTorqueControlMode,this);
    this->addOperation("setJointPositionControlMode",&KRLTool::setJointPositionControlMode,this);
    this->addOperation("setCartesianImpedanceControlMode",&KRLTool::setCartesianImpedanceControlMode,this);

    this->addOperation("setJointImpedanceControlModeROSService",&KRLTool::setJointImpedanceControlModeROSService,this);
    this->addOperation("setJointTorqueControlModeROSService",&KRLTool::setJointTorqueControlModeROSService,this);
    this->addOperation("setJointPositionControlModeROSService",&KRLTool::setJointPositionControlModeROSService,this);
    this->addOperation("setCartesianImpedanceControlModeROSService",&KRLTool::setCartesianImpedanceControlModeROSService,this);

    this->addOperation("getCurrentControlModeROSService",&KRLTool::getCurrentControlModeROSService,this);
    this->addOperation("getCurrentControlMode",&KRLTool::getCurrentControlMode,this);

    this->addOperation("resetJointImpedanceGains",&KRLTool::resetJointImpedanceGains,this);
    this->addOperation("setStiffnessZero",&KRLTool::setStiffnessZero,this);
    this->addOperation("PTP",&KRLTool::PTP,this);
    this->addOperation("PTP_REL",&KRLTool::PTP_REL,this);
    this->addOperation("LIN",&KRLTool::LIN,this);
    this->addOperation("LIN_REL",&KRLTool::LIN_REL,this);
    this->addOperation("printBool",&KRLTool::printBool,this);
    this->addOperation("printInt",&KRLTool::printInt,this);
    this->addOperation("printReal",&KRLTool::printReal,this);
    this->addOperation("printAll",&KRLTool::printAll,this);
    this->addOperation("setTool",&KRLTool::setTool,this);
    this->addOperation("setBase",&KRLTool::setBase,this);
    this->addOperation("sendSTOP2",&KRLTool::sendSTOP2,this);
    this->addOperation("unsetSTOP2",&KRLTool::unsetSTOP2,this);
    this->addOperation("setVELPercent",&KRLTool::setVELPercent,this);
    this->addOperation("sendSTOP2_srv",&KRLTool::sendSTOP2_srv,this);
    this->addOperation("unsetSTOP2_srv",&KRLTool::unsetSTOP2_srv,this);
    this->addOperation("setMaxVelPercent",&KRLTool::setMaxVelPercent,this);
    this->addAttribute("doUpdate",do_update);

    for(unsigned int i=0;i<FRI_USER_SIZE;i++)
    {
        fromKRL.intData[i] = toKRL.intData[i] =  0;
        fromKRL.realData[i] = toKRL.realData[i] = 0.0;
    }
    fromKRL.boolData = 0;
    toKRL.boolData = 0;
    // Add action server ports to this task's root service
    ptp_action_server_.addPorts(this->provides("PTP"));
    lin_action_server_.addPorts(this->provides("LIN"));

    // Bind action server goal and cancel callbacks (see below)
    ptp_action_server_.registerGoalCallback(boost::bind(&KRLTool::PTPgoalCallback, this, _1));
    ptp_action_server_.registerCancelCallback(boost::bind(&KRLTool::PTPcancelCallback, this, _1));
    lin_action_server_.registerGoalCallback(boost::bind(&KRLTool::LINgoalCallback, this, _1));
    lin_action_server_.registerCancelCallback(boost::bind(&KRLTool::LINcancelCallback, this, _1));
}
// Called by ptp_action_server_ when a new goal is received
void KRLTool::PTPgoalCallback(PTPGoalHandle gh)
{
    if(gh.getGoal()->ptp_goal_rad.size() != LBR_MNJ)
    {
        log(Error) << "ptp goal size is wrong ("<<gh.getGoal()->ptp_goal_rad.size()<<", but should be "<<LBR_MNJ<<")"<<endlog();
        return;
    }
    if(gh.getGoal()->ptp_goal_rad.size() != gh.getGoal()->ptp_mask.size())
    {
        log(Warning) << "The mask is not the same size as the goal ("<<gh.getGoal()->ptp_goal_rad.size()<<" vs "<<gh.getGoal()->ptp_mask.size()<<")"<<endlog();
        return;
    }
    std::vector<double> ptp_cmd(LBR_MNJ,0.0);
    std::vector<double> ptp_mask(LBR_MNJ,0);
    for(int i=0;i<ptp_cmd.size();++i)
    {
        if(gh.getGoal()->ptp_mask[i])
        {
            ptp_mask[i] = 1;
            ptp_cmd[i] = gh.getGoal()->ptp_goal_rad[i];
        }
    }
    if(gh.getGoal()->use_relative)
    {
        log(Warning) << "Sending new PTP_REL Command "<<endlog();
    }
    else
    {
        log(Warning) << "Sending new PTP Command "<<endlog();
    }
    this->PointToPoint(ptp_cmd,ptp_mask,true,gh.getGoal()->use_relative,gh.getGoal()->vel_percent);
    gh.setAccepted();
    ptp_current_gh = gh;
}

void KRLTool::PTPcancelCallback(PTPGoalHandle gh)
{
    log(Warning) << "You asked to cancel the current PTP goal"<<endlog();
    if(ptp_current_gh == gh && ptp_current_gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE) {
      ptp_current_gh.setCanceled(ptp_result);
    }
}

void KRLTool::LINgoalCallback(LINGoalHandle gh)
{
    if(gh.getGoal()->use_relative)
    {
        log(Warning) << "Sending new LIN_REL Command "<<endlog();
    }
    else
    {
        log(Warning) << "Sending new LIN Command "<<endlog();
    }
    Linear(gh.getGoal()->XYZ,gh.getGoal()->RPY,gh.getGoal()->use_relative);
    gh.setAccepted();
    lin_current_gh = gh;
}

void KRLTool::LINcancelCallback(LINGoalHandle gh)
{
    log(Warning) << "You asked to cancel the current LIN goal"<<endlog();
    if(lin_current_gh == gh && lin_current_gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE) {
      lin_current_gh.setCanceled(lin_result);
    }
}

void KRLTool::LIN(const geometry_msgs::Vector3& XYZ_meters,
    const geometry_msgs::Vector3& RPY_rad)
{
    Linear(XYZ_meters,RPY_rad,false);
}

void KRLTool::LIN_REL(const geometry_msgs::Vector3& XYZ_meters,
    const geometry_msgs::Vector3& RPY_rad)
{
    Linear(XYZ_meters,RPY_rad,true);
}

void KRLTool::Linear(const geometry_msgs::Vector3& XYZ_meters,
    const geometry_msgs::Vector3& RPY_rad,
    bool use_lin_rel)
{
    bool use_radians = true;

    double conv = 1.0;

    if(use_radians)
        conv = 180.0/3.14159265359;

    setBit(toKRL.boolData,LIN_CMD,true);
    toKRL.intData[LIN_CMD_TYPE] = use_lin_rel;
    toKRL.realData[X] = XYZ_meters.x * 1000.0;
    toKRL.realData[Y] = XYZ_meters.y * 1000.0;
    toKRL.realData[Z] = XYZ_meters.z * 1000.0;
    // WARNING: A is rotation around Z , B around Y, C around X
    toKRL.realData[A] = RPY_rad.z * conv;
    toKRL.realData[B] = RPY_rad.y * conv;
    toKRL.realData[C] = RPY_rad.x * conv;
    doUpdate();
}

void KRLTool::sendSTOP2()
{
    setBit(toKRL.boolData,STOP2,true);
    doUpdate();
}

void KRLTool::unsetSTOP2()
{
    setBit(toKRL.boolData,STOP2,false);
    doUpdate();
}

bool KRLTool::sendSTOP2_srv(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
    sendSTOP2();
    return true;
}

bool KRLTool::unsetSTOP2_srv(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
    unsetSTOP2();
    return true;
}

void KRLTool::resetJointImpedanceGains()
{
    for(unsigned int i=0;i<cmd.stiffness.size();++i){
        cmd.stiffness[i] = 1000.;
        cmd.damping[i] = 0.7;
    }
    do_send_imp_cmd = true;
}
void KRLTool::setStiffnessZero()
{
    for(unsigned int i=0;i<cmd.damping.size();++i){
        cmd.stiffness[i] = 0.0;
        cmd.damping[i] = 0.0;
    }
    do_send_imp_cmd = true;
    is_joint_torque_control_mode = true;
}

bool KRLTool::isJointPositionMode()
{
    return static_cast<FRI_CTRL>(fromKRL.intData[CONTROL_MODE]) == 10*FRI_CTRL_POSITION;
}
bool KRLTool::isJointTorqueMode()
{
    return is_joint_torque_control_mode;
}
bool KRLTool::isJointImpedanceMode()
{
    return static_cast<FRI_CTRL>(fromKRL.intData[CONTROL_MODE]) == 10*FRI_CTRL_JNT_IMP;
}

bool KRLTool::isCartesianImpedanceMode()
{
    return static_cast<FRI_CTRL>(fromKRL.intData[CONTROL_MODE]) == 10*FRI_CTRL_CART_IMP;
}

void KRLTool::setJointPositionControlMode()
{
    toKRL.intData[CONTROL_MODE] = 10*FRI_CTRL_POSITION;
    setBit(toKRL.boolData,SET_CONTROL_MODE,true);
    doUpdate();
}

void KRLTool::setJointImpedanceControlMode()
{
    toKRL.intData[CONTROL_MODE] = FRI_CTRL_JNT_IMP*10;
    setBit(toKRL.boolData,SET_CONTROL_MODE,true);
    doUpdate();
}

void KRLTool::setCartesianImpedanceControlMode()
{
    toKRL.intData[CONTROL_MODE] = FRI_CTRL_CART_IMP*10;
    setBit(toKRL.boolData,SET_CONTROL_MODE,true);
    doUpdate();
}

void KRLTool::setJointTorqueControlMode()
{
    setJointImpedanceControlMode();
    setStiffnessZero();
}

bool KRLTool::configureHook()
{
    for(unsigned i=0;i<FRI_USER_SIZE;++i)
    {
        intDataToKRL.data.push_back(0);
        realDataToKRL.data.push_back(0.0);
        intDataFromKRL.data.push_back(0);
        realDataFromKRL.data.push_back(0.0);
        boolDataFromKRL.data.push_back(0);
    }
    port_intDataToKRL_ros.createStream(rtt_roscomm::topic(getName()+"/intDataToKRL"));
    port_realDataToKRL_ros.createStream(rtt_roscomm::topic(getName()+"/realDataToKRL"));

    port_intDataFromKRL_ros.createStream(rtt_roscomm::topic(getName()+"/intDataFromKRL"));
    port_realDataFromKRL_ros.createStream(rtt_roscomm::topic(getName()+"/realDataFromKRL"));
    port_boolDataFromKRL_ros.createStream(rtt_roscomm::topic(getName()+"/boolDataFromKRL"));

    boost::shared_ptr<rtt_rosservice::ROSService> rosservice
        = this->getProvider<rtt_rosservice::ROSService>("rosservice");

    if(rosservice)
    {
        rosservice->connect("sendSTOP2_srv",this->getName()+"/send_stop2","std_srvs/Empty");
        rosservice->connect("unsetSTOP2_srv",this->getName()+"/unset_stop2","std_srvs/Empty");
        rosservice->connect("setMaxVelPercent",this->getName()+"/set_max_vel_percent","krl_msgs/SetMaxVelPercent");
    }
    else
    {
        RTT::log(RTT::Warning) << "ROSService not available" << RTT::endlog();
    }

    // lin_action_server_.initialize();
    // ptp_action_server_.initialize();
    // int n=30;
    // while(!lin_action_server_.ready() && !ptp_action_server_.ready())
    // {
    //     if(!--n) break;
    //     usleep(1E6);
    // }

    return true;
}

bool KRLTool::setMaxVelPercent(krl_msgs::SetMaxVelPercentRequest& req,krl_msgs::SetMaxVelPercentResponse& resp)
{
    setVELPercent(req.max_vel_percent);
    return true;
}

bool KRLTool::getCurrentControlModeROSService(std_srvs::TriggerRequest& req,std_srvs::TriggerResponse& resp)
{
    resp.success = true;
    switch(static_cast<FRI_CTRL>(fromKRL.intData[CONTROL_MODE]))
    {
        case FRI_CTRL_POSITION*10:
            resp.message = "Joint Position Mode - FRI_CTRL_POSITION";
            break;
        case FRI_CTRL_JNT_IMP*10:
            resp.message = "Joint Impedance Mode - FRI_CTRL_JNT_IMP";
            break;
        case FRI_CTRL_CART_IMP*10:
            resp.message = "Cartesian Impedance Mode - FRI_CTRL_CART_IMP";
            break;
        case FRI_CTRL_OTHER:
            resp.message = "FRI_CTRL_OTHER";
            break;
        default:
            resp.success = false;
            resp.message = "Wrong Control Mode";
            log(Info) << "fromKRL.intData[CONTROL_MODE] = "<<static_cast<FRI_CTRL>(fromKRL.intData[CONTROL_MODE])<<endlog();
    }
    log(Info) << "KRLTool::getCurrentControlModeROSService - "<<resp.message << endlog();
    return resp.success;
}

bool KRLTool::getCurrentControlMode()
{
    std_srvs::TriggerRequest req;
    std_srvs::TriggerResponse resp;
    return getCurrentControlModeROSService(req,resp);
}

bool KRLTool::setJointImpedanceControlModeROSService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& resp)
{
    log(Info) << "KRLTool::setJointImpedanceControlModeROSService" << endlog();
    setJointImpedanceControlMode();
}
bool KRLTool::setJointPositionControlModeROSService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& resp)
{
    log(Info) << "KRLTool::setJointPositionControlModeROSService" << endlog();
    setJointPositionControlMode();
}
bool KRLTool::setJointTorqueControlModeROSService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& resp)
{
    log(Info) << "KRLTool::setJointTorqueControlModeROSService" << endlog();
    setJointTorqueControlMode();
}
bool KRLTool::setCartesianImpedanceControlModeROSService(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
    log(Info) << "KRLTool::setCartesianImpedanceControlModeROSService" << endlog();
    setCartesianImpedanceControlMode();
}
void KRLTool::PTP_REL(const std::vector<double>& ptp,
    const std::vector<double>& mask,
    bool use_radians,
    double vel_ptp)
{
    PointToPoint(ptp,mask,use_radians,true,vel_ptp);
}
void KRLTool::PTP(const std::vector<double>& ptp,
    const std::vector<double>& mask,
    bool use_radians,
    double vel_ptp)
{
    PointToPoint(ptp,mask,use_radians,false,vel_ptp);
}

void KRLTool::PointToPoint(const std::vector<double>& ptp,
    const std::vector<double>& mask,
    bool use_radians,
    bool use_ptp_rel,
    double vel_ptp)
{
    if(ptp.size() != LBR_MNJ)
    {
        log(Error) << "KRLTool::PTP : ptp vector size is "<<ptp.size()<<" but should be "<<LBR_MNJ<< endlog();
        return;
    }

    if(mask.size() != LBR_MNJ)
    {
        log(Error) << "KRLTool::PTP : mask size is "<<mask.size()<<" but should be "<<LBR_MNJ<< endlog();
        return;
    }

    double conv = 1.0;

    if(use_radians)
        conv = 180.0/3.14159265359;

    toKRL.intData[PTP_CMD_TYPE] = use_ptp_rel;
    toKRL.realData[A1] = conv * ptp[0];
    toKRL.realData[A2] = (use_ptp_rel ? 0.0:90.0) + conv * ptp[1];
    toKRL.realData[E1] = conv * ptp[2];
    toKRL.realData[A3] = conv * ptp[3];
    toKRL.realData[A4] = conv * ptp[4];
    toKRL.realData[A5] = conv * ptp[5];
    toKRL.realData[A6] = conv * ptp[6];
    setBit(toKRL.boolData,MASK_0,mask[0]);
    setBit(toKRL.boolData,MASK_1,mask[1]);
    setBit(toKRL.boolData,MASK_2,mask[2]);
    setBit(toKRL.boolData,MASK_3,mask[3]);
    setBit(toKRL.boolData,MASK_4,mask[4]);
    setBit(toKRL.boolData,MASK_5,mask[5]);
    setBit(toKRL.boolData,MASK_6,mask[6]);

    setBit(toKRL.boolData,PTP_CMD,true);
    doUpdate();
}

void KRLTool::setBase(int base_number)
{
  toKRL.intData[BASE] = base_number;
  setBit(toKRL.boolData,SET_BASE,true);
  doUpdate();
}
void KRLTool::setTool(int tool_number)
{
  toKRL.intData[TOOL] = tool_number;
  setBit(toKRL.boolData,SET_TOOL,true);
  doUpdate();
}
void KRLTool::setVELPercent(float vel_percent)
{
  if( 0 <= vel_percent && vel_percent <= 100)
  {
    toKRL.realData[VEL_PERCENT] = vel_percent;
    setBit(toKRL.boolData,SET_VEL,true);
  }else{
    log(Error) << "Max velocity must be between [0:100]"<< endlog();
  }
}
void KRLTool::printBool()
{
    cout <<"toKRL.boolData   : [ ";for(int i=0;i<FRI_USER_SIZE;++i) cout << getBit(toKRL.boolData,i) <<" ";cout << "]" <<endl;
    cout <<"fromKRL.boolData : [ ";for(int i=0;i<FRI_USER_SIZE;++i) cout << getBit(fromKRL.boolData,i) <<" ";cout << "]" <<endl;
}
void KRLTool::printInt()
{
    cout <<"toKRL.intData :   [ ";for(int i=0;i<FRI_USER_SIZE;++i) cout << toKRL.intData[i] <<" ";cout << "]" <<endl;
    cout <<"fromKRL.intData : [ ";for(int i=0;i<FRI_USER_SIZE;++i) cout << fromKRL.intData[i] <<" ";cout << "]" <<endl;
}
void KRLTool::printReal()
{
    cout <<"toKRL.realData :   [ ";for(int i=0;i<FRI_USER_SIZE;++i) cout << toKRL.realData[i] <<" ";cout << "]" <<endl;
    cout <<"fromKRL.realData : [ ";for(int i=0;i<FRI_USER_SIZE;++i) cout << fromKRL.realData[i] <<" ";cout << "]" <<endl;
}
void KRLTool::printAll()
{
    printBool();
    printInt();
    printReal();
}

bool KRLTool::startHook() {
  // Start action server
  ptp_action_server_.start();
  lin_action_server_.start();
  return true;
}

void KRLTool::updateHook()
{
    static bool has_sent_cmd = false;
    // Incoming ROS Int message
//     if(port_intDataToKRL_ros.read(intDataToKRL) == NewData)
//     {
//         for(unsigned i=0;i<FRI_USER_SIZE && i<intDataToKRL.data.size();++i)
//             if(intDataToKRL.data[i] != lwr::ROS_MASK_NO_UPDATE)
//                 toKRL.intData[i] = static_cast<fri_int32_t>(intDataToKRL.data[i]);
//         doUpdate();
//     }
//
//     // Incoming ROS Float message
//     if(port_realDataToKRL_ros.read(realDataToKRL) == NewData)
//     {
//         for(unsigned i=0;i<FRI_USER_SIZE && i<realDataToKRL.data.size();++i)
//             if(realDataToKRL.data[i] != lwr::ROS_MASK_NO_UPDATE)
//                 toKRL.realData[i] = static_cast<fri_float_t>(realDataToKRL.data[i]);
//         doUpdate();
//     }

    // To KRL
    if(do_update)
    {

        if(getBit(fromKRL.boolData,KRL_LOOP_REQUESTED) && has_sent_cmd)
        {
            noUpdate();
            has_sent_cmd = false;

            for(unsigned int i=0;i<FRI_USER_SIZE;++i)
            {
                // Reset bits if they have been acked .Special case STOP2
                if(getBit(fromKRL.boolData,i) && i != STOP2 && i != SET_VEL)
                    setBit(toKRL.boolData,i,false);
            }

            if(ptp_current_gh.isValid() && ptp_current_gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
            {
                ptp_current_gh.setSucceeded(ptp_result);
            }
            if(lin_current_gh.isValid() && lin_current_gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
            {
                lin_current_gh.setSucceeded(lin_result);
            }
            // printBool();
            // log(Info) << "----- ACKED   -----" << endlog();

        }
        else if(getBit(fromKRL.boolData,KRL_LOOP_REQUESTED) && !has_sent_cmd)
        {
            // special case, bug or error, lets write 00
            // setBit(toKRL.boolData,KRL_LOOP_REQUESTED,false);
            // printAll();
            // log(Error) << "getBit(fromKRL.boolData,KRL_LOOP_REQUESTED) && !has_sent_cmd THIS SHOULD NOT HAPPEND"<< endlog();
        }
        else
        {
            // Request an update
            setBit(toKRL.boolData,KRL_LOOP_REQUESTED,true);
            // printBool();
            // cout <<"----- WRITING -----" << endl;
            has_sent_cmd = true;
        }

    }
    else
    {
        for(unsigned int i=0;i<FRI_USER_SIZE;++i)
        {
            if(getBit(fromKRL.boolData,i) && i != STOP2 && i != SET_VEL)
                setBit(toKRL.boolData,i,false);
        }
    }

    if(getBit(fromKRL.boolData,SET_VEL))
    {
        setBit(toKRL.boolData,SET_VEL,false);
        // printBool();
        // log(Info) << "----- VEL ACKED -----" << endlog();
    }

    port_toKRL.write(toKRL);

    // Joint Impedance Commands
    if(do_send_imp_cmd){
        port_JointImpedanceCommand.write(cmd);
        do_send_imp_cmd = false;
    }

    port_fromKRL.read(fromKRL); // Should be after the first command !

//         /* To ROS for plotting */
//     for(int i=0;i<FRI_USER_SIZE;++i)
//         intDataFromKRL.data[i] = fromKRL.intData[i];
//     for(int i=0;i<FRI_USER_SIZE;++i)
//         realDataFromKRL.data[i] = fromKRL.realData[i];
//     for(int i=0;i<FRI_USER_SIZE;++i)
//         boolDataFromKRL.data[i] = getBit(fromKRL.boolData , i);
//
//     port_intDataFromKRL_ros.write(intDataFromKRL);
//     port_realDataFromKRL_ros.write(realDataFromKRL);
//     port_boolDataFromKRL_ros.write(boolDataFromKRL);

}

}
