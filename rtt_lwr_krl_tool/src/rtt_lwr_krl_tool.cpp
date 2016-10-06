// Copyright 2015 ISIR
// Author: Antoine Hoarau <hoarau.robotics@gmail.com>

#include "rtt_lwr_krl_tool/rtt_lwr_krl_tool.hpp"

namespace lwr{
using namespace RTT;
using namespace krl;
using namespace RTT::os;

KRLTool::KRLTool(const std::string& name):
TaskContext(name),
do_send_imp_cmd(false),
// startPTP(LBR_MNJ),
// startLIN(3),
is_joint_torque_control_mode(false)
{
    this->ports()->addPort("toKRL",port_toKRL).doc("Struct defined in friComm.h to send to the KRL Program");
    this->ports()->addEventPort("fromKRL",port_fromKRL).doc("Struct defined in friComm.h to read from KRL Program");
    this->ports()->addPort("JointImpedanceCommand",port_JointImpedanceCommand).doc("");

    this->ports()->addPort("intDataToKRL_ros",port_intDataToKRL_ros).doc("");
    this->ports()->addPort("intDataFromKRL_ros",port_intDataFromKRL_ros).doc("");
    this->ports()->addPort("realDataToKRL_ros",port_realDataToKRL_ros).doc("");
    this->ports()->addPort("realDataFromKRL_ros",port_realDataFromKRL_ros).doc("");
    this->ports()->addPort("boolDataFromKRL_ros",port_boolDataFromKRL_ros).doc("");

    //FT Sensor Monitoring
    this->ports()->addPort("ft_sensor_in",port_ft_sensor).doc("");

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

    this->provides("FRI")->addOperation("open",&KRLTool::FRIOpen,this);
    this->provides("FRI")->addOperation("start",&KRLTool::FRIStart,this);
    this->provides("FRI")->addOperation("stop",&KRLTool::FRIStop,this);
    this->provides("FRI")->addOperation("close",&KRLTool::FRIClose,this);

    this->addOperation("printBool",&KRLTool::printBool,this);
    this->addOperation("printInt",&KRLTool::printInt,this);
    this->addOperation("printReal",&KRLTool::printReal,this);
    this->addOperation("printAll",&KRLTool::printAll,this);
    this->addOperation("cancelGoals",&KRLTool::cancelGoals,this);
    this->addOperation("cancelMotion",&KRLTool::cancelMotion,this);
    this->addOperation("setTool",&KRLTool::setTool,this);
    this->addOperation("setBase",&KRLTool::setBase,this);
    this->addOperation("sendSTOP2",&KRLTool::sendSTOP2,this);
    this->addOperation("unsetSTOP2",&KRLTool::unsetSTOP2,this);
    this->addOperation("setVELPercent",&KRLTool::setVELPercent,this);
    this->addOperation("sendSTOP2_srv",&KRLTool::sendSTOP2_srv,this);
    this->addOperation("unsetSTOP2_srv",&KRLTool::unsetSTOP2_srv,this);
    this->addOperation("setMaxVelPercent_srv",&KRLTool::setMaxVelPercent_srv,this);
    this->addOperation("setTool_srv",&KRLTool::setTool_srv,this);
    this->addOperation("setBase_srv",&KRLTool::setBase_srv,this);
    this->addOperation("setToolBase_srv",&KRLTool::setToolBase_srv,this);

    this->addOperation("resetData",&KRLTool::resetData,this);

    // this->provides("PTP")->addOperation("move")

    for(unsigned int i=0;i<FRI_USER_SIZE;i++)
    {
        fromKRL.intData[i] = toKRL.intData[i] =  0;
        fromKRL.realData[i] = toKRL.realData[i] = 0.0;
    }

    addNoAckNeededVar(STOP2);
    addNoAckNeededVar(SET_VEL);
    addNoAckNeededVar(CANCEL_MOTION);

    fromKRL.boolData = 0;
    toKRL.boolData = 0;

}

bool KRLTool::sendFRICommand(int cmd,bool wait_until_done)
{
    toKRL.intData[FRI_CMD] = cmd;
    if(wait_until_done)
    {
        port_toKRL.write(toKRL);
        port_fromKRL.read(fromKRL);
        TimeService::ticks timestamp = TimeService::Instance()->getTicks();
        while(fromKRL.intData[FRI_CMD] != cmd)
        {
            if(TimeService::Instance()->secondsSince( timestamp ) > RTT::Seconds(5.0))
            {
                log(Error) << "\n\n\n Could not execute FRI Command , please check teach pendant \n\n" << endlog();
                return false;
            }
            port_fromKRL.read(fromKRL);
            usleep(500);
            //log(Info) << "----- Waiting -----" << endlog();
        }
        toKRL.intData[FRI_CMD] = 0;
        port_toKRL.write(toKRL);
    }
    return true;
}

bool KRLTool::FRIOpen(int period_ms)
{
    if(1 < period_ms && period_ms <= 1000)
        toKRL.intData[FRI_PERIOD_MS] = period_ms;
    return sendFRICommand(FRI_OPEN);
}
bool KRLTool::FRIStart()
{
    return sendFRICommand(FRI_START);
}
bool KRLTool::FRIStop()
{
    return sendFRICommand(FRI_STOP);
}
bool KRLTool::FRIClose()
{
    return sendFRICommand(FRI_CLOSE);
}

bool KRLTool::resetData()
{
    toKRL.boolData = 0;
    for (size_t i = 0; i < FRI_USER_SIZE; i++) {
        toKRL.intData[i] = toKRL.realData[i] = 0;
    }
    bool wait_until_done = true;
    toKRL.intData[RESET_ALL_DATA] = 1;

    if(wait_until_done)
    {
      port_toKRL.write(toKRL);
      port_fromKRL.read(fromKRL);
      TimeService::ticks timestamp = TimeService::Instance()->getTicks();
      while(fromKRL.intData[RESET_ALL_DATA] != 1)
      {
          if(TimeService::Instance()->secondsSince( timestamp ) > RTT::Seconds(5.0))
          {
              log(Error) << "\n\n\n Could not reset FRI data, please restart the KRL script. \n\n" << endlog();
              return false;
          }
          port_fromKRL.read(fromKRL);
          usleep(500);
          //log(Info) << "----- Waiting -----" << endlog();
      }
      toKRL.boolData = 0;
      for (size_t i = 0; i < FRI_USER_SIZE; i++) {
          toKRL.intData[i] = toKRL.realData[i] = 0;
      }
      port_toKRL.write(toKRL);
    }
    return true;
}

// Called by ptp_action_server_ when a new goal is received
void KRLTool::PTPgoalCallback(PTPGoalHandle gh)
{
    if(lin_current_gh.getGoal() && lin_current_gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE
    || ptp_current_gh.getGoal() && ptp_current_gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
    {
      gh.setRejected();
      log(Warning) << "Rejecting PTP Goal "<<gh.getGoal()<<" because another one is active"<<endlog();
      return;
    }
    std::vector<double> ptp_cmd(LBR_MNJ,0.0);
    std::vector<bool> ptp_mask(LBR_MNJ,false);

    if(gh.getGoal()->ptp_mask.size() != gh.getGoal()->ptp_goal.size())
    {
        log(Error) << "ptp command and mask sizes don't match ("<<gh.getGoal()->ptp_goal.size()<<"!="<<gh.getGoal()->ptp_mask.size()<<")"<<endlog();
        return;
    }

    for(int i=0;i<ptp_cmd.size()
    && i<gh.getGoal()->ptp_mask.size()
    && i<gh.getGoal()->ptp_goal.size();++i)
    {
        if(gh.getGoal()->ptp_mask[i])
        {
            ptp_mask[i] = true;
            ptp_cmd[i] = gh.getGoal()->ptp_goal[i];
        }
    }

    log(Warning) << "Sending new PTP"<<(gh.getGoal()->use_relative ? "_REL":"")
    <<" Command using "<<(gh.getGoal()->ptp_input_type == JOINT ? "Joints":" Cartesian")<<endlog();

    this->PointToPoint(
        ptp_cmd,
        ptp_mask,
        gh.getGoal()->XYZ,
        gh.getGoal()->RPY,
        gh.getGoal()->XYZ_mask,
        gh.getGoal()->RPY_mask,
        gh.getGoal()->use_radians,
        gh.getGoal()->ptp_input_type,
        gh.getGoal()->use_relative,
        gh.getGoal()->vel_percent
    );

    gh.setAccepted();
    ptp_current_gh = gh;
}

void KRLTool::PTPcancelCallback(PTPGoalHandle gh)
{
    log(Warning) << "You asked to cancel the current PTP goal"<<endlog();
    cancelMotion();
}

void KRLTool::cancelGoals()
{
    if(ptp_current_gh.isValid() && ptp_current_gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
    {
        log(Warning) << "Cancelling PTP goal"<<endlog();
        ptp_current_gh.setCanceled();
    }
    if(lin_current_gh.isValid() && lin_current_gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
    {
        log(Warning) << "Cancelling LIN goal"<<endlog();
        lin_current_gh.setCanceled();
    }
}

void KRLTool::LINgoalCallback(LINGoalHandle gh)
{
    if(lin_current_gh.isValid() && lin_current_gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE
    || ptp_current_gh.isValid() && ptp_current_gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
    {
      gh.setRejected();
      log(Warning) << "Rejecting LIN Goal "<<gh.getGoal()<<" because another one is active"<<endlog();
      return;
    }

    log(Warning) << "Sending new LIN"<<(gh.getGoal()->use_relative ? "_REL":"")<<" Command in "<<(gh.getGoal()->in_tool_frame ? "tool":"base")<<" frame"<<endlog();

    this->Linear(
        gh.getGoal()->XYZ,
        gh.getGoal()->RPY,
        gh.getGoal()->XYZ_mask,
        gh.getGoal()->RPY_mask,
        gh.getGoal()->use_relative,
        gh.getGoal()->vel_percent,
        gh.getGoal()->in_tool_frame
    );
    gh.setAccepted();
    lin_current_gh = gh;
}

void KRLTool::LINcancelCallback(LINGoalHandle gh)
{
    log(Warning) << "You asked to cancel the current LIN goal"<<endlog();
    cancelMotion();
}

void KRLTool::Linear(
    const geometry_msgs::Vector3& XYZ_meters,
    const geometry_msgs::Vector3& RPY_rad,
    const geometry_msgs::Vector3& XYZ_mask,
    const geometry_msgs::Vector3& RPY_mask,
    bool use_rel,
    double vel_percent,
    bool in_tool_frame)
{
    bool use_radians = true;

    double conv = 1.0;

    if(use_radians)
        conv = 180.0/3.14159265359;

    setBit(toKRL.boolData,LIN_CMD,true);
    setBit(toKRL.boolData,PTP_CMD,false);

    toKRL.realData[X] = XYZ_meters.x * 1000.0;
    toKRL.realData[Y] = XYZ_meters.y * 1000.0;
    toKRL.realData[Z] = XYZ_meters.z * 1000.0;
    // WARNING: A is rotation around Z , B around Y, C around X
    toKRL.realData[A] = RPY_rad.z * conv;
    toKRL.realData[B] = RPY_rad.y * conv;
    toKRL.realData[C] = RPY_rad.x * conv;

    setBit(toKRL.boolData,MASK_0,XYZ_mask.x);
    setBit(toKRL.boolData,MASK_1,XYZ_mask.y);
    setBit(toKRL.boolData,MASK_2,XYZ_mask.z);
    setBit(toKRL.boolData,MASK_3,RPY_mask.z); // A
    setBit(toKRL.boolData,MASK_4,RPY_mask.y); // B
    setBit(toKRL.boolData,MASK_5,RPY_mask.x); // C

    toKRL.intData[CMD_INPUT_TYPE] = (in_tool_frame ? CARTESIAN_IN_TOOL:CARTESIAN_IN_BASE);
    toKRL.intData[USE_RELATIVE] = use_rel;

    if(in_tool_frame && !use_rel)
    {
        log(Error) << "Moving in tool frame only works with Relative movements !" << endlog();
        return;
    }

    if( 0 <= vel_percent && vel_percent <= 100.0)
    {
        toKRL.realData[CMD_VEL_PERCENT] = vel_percent / 2.0; // WARNING : 100% is 2m/s which is really too fast as hell
    }
    else
    {
        toKRL.realData[CMD_VEL_PERCENT] = 1.0;
        log(Error) << "Vel percent should be between [0:100.], you provided ("<<vel_percent<<"), setting to 1% instead."<<endlog();
    }
    setBit(toKRL.boolData,KRL_LOOP_REQUESTED,true);
}

void KRLTool::sendSTOP2()
{
    setBit(toKRL.boolData,STOP2,true);
}

void KRLTool::unsetSTOP2()
{
    setBit(toKRL.boolData,STOP2,false);
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
    setBit(toKRL.boolData,KRL_LOOP_REQUESTED,true);
}

void KRLTool::setJointImpedanceControlMode()
{
    toKRL.intData[CONTROL_MODE] = FRI_CTRL_JNT_IMP*10;
    setBit(toKRL.boolData,SET_CONTROL_MODE,true);
    setBit(toKRL.boolData,KRL_LOOP_REQUESTED,true);
}

void KRLTool::setCartesianImpedanceControlMode()
{
    toKRL.intData[CONTROL_MODE] = FRI_CTRL_CART_IMP*10;
    setBit(toKRL.boolData,SET_CONTROL_MODE,true);
    setBit(toKRL.boolData,KRL_LOOP_REQUESTED,true);
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
    // port_intDataToKRL_ros.createStream(rtt_roscomm::topic(getName()+"/intDataToKRL"));
    // port_realDataToKRL_ros.createStream(rtt_roscomm::topic(getName()+"/realDataToKRL"));

    port_intDataFromKRL_ros.createStream(rtt_roscomm::topic(getName()+"/intDataFromKRL"));
    port_realDataFromKRL_ros.createStream(rtt_roscomm::topic(getName()+"/realDataFromKRL"));
    port_boolDataFromKRL_ros.createStream(rtt_roscomm::topic(getName()+"/boolDataFromKRL"));

    boost::shared_ptr<rtt_rosservice::ROSService> rosservice
        = this->getProvider<rtt_rosservice::ROSService>("rosservice");

    if(rosservice)
    {
        rosservice->connect("sendSTOP2_srv",this->getName()+"/send_stop2","std_srvs/Empty");
        rosservice->connect("unsetSTOP2_srv",this->getName()+"/unset_stop2","std_srvs/Empty");
        rosservice->connect("setMaxVelPercent_srv",this->getName()+"/set_max_vel_percent","krl_msgs/SetMaxVelPercent");
        rosservice->connect("setTool_srv",this->getName()+"/set_tool","krl_msgs/SetTool");
        rosservice->connect("setBase_srv",this->getName()+"/set_base","krl_msgs/SetBase");
        rosservice->connect("setToolBase_srv",this->getName()+"/set_tool_base","krl_msgs/SetToolBase");
    }
    else
    {
        RTT::log(RTT::Warning) << "ROSService not available" << RTT::endlog();
    }

    // Add action server ports to this task's root service
    ptp_action_server_.addPorts(this->provides("PTP"));
    lin_action_server_.addPorts(this->provides("LIN"));

    // Bind action server goal and cancel callbacks (see below)
    ptp_action_server_.registerGoalCallback(boost::bind(&KRLTool::PTPgoalCallback, this, _1));
    ptp_action_server_.registerCancelCallback(boost::bind(&KRLTool::PTPcancelCallback, this, _1));
    lin_action_server_.registerGoalCallback(boost::bind(&KRLTool::LINgoalCallback, this, _1));
    lin_action_server_.registerCancelCallback(boost::bind(&KRLTool::LINcancelCallback, this, _1));

    return true;
}
bool KRLTool::setToolBase_srv(krl_msgs::SetToolBaseRequest& req,krl_msgs::SetToolBaseResponse& resp)
{
    setTool(req.tool_number);
    setBase(req.base_number);
    return true;
}
bool KRLTool::setTool_srv(krl_msgs::SetToolRequest& req,krl_msgs::SetToolResponse& resp)
{
    setTool(req.tool_number);
    return true;
}
bool KRLTool::setBase_srv(krl_msgs::SetBaseRequest& req,krl_msgs::SetBaseResponse& resp)
{
    setBase(req.base_number);
    return true;
}
bool KRLTool::setMaxVelPercent_srv(krl_msgs::SetMaxVelPercentRequest& req,krl_msgs::SetMaxVelPercentResponse& resp)
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

void KRLTool::PointToPoint(
    const std::vector<double>& ptp,
    const std::vector<bool>& mask,
    const geometry_msgs::Vector3& XYZ,
    const geometry_msgs::Vector3& RPY,
    const geometry_msgs::Vector3& XYZ_mask,
    const geometry_msgs::Vector3& RPY_mask,
    bool use_radians,
    int ptp_input_type,
    bool use_rel,
    double vel_percent)
{
    if(ptp_input_type == JOINT && ptp.size() != LBR_MNJ)
    {
        log(Error) << "KRLTool::PTP : ptp vector size is "<<ptp.size()<<" but should be "<<LBR_MNJ<< endlog();
        return;
    }

    if(ptp_input_type == JOINT && mask.size() != LBR_MNJ)
    {
        log(Error) << "KRLTool::PTP : mask size is "<<mask.size()<<" but should be "<<LBR_MNJ<< endlog();
        return;
    }

    double conv = 1.0;

    if(use_radians)
        conv = 180.0/3.14159265359;

    switch(ptp_input_type)
    {
        case JOINT:
            toKRL.realData[A1] = conv * ptp[0];
            toKRL.realData[A2] = (use_rel ? 0.0:90.0) + conv * ptp[1];
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
        break;

        case CARTESIAN_IN_BASE:
            toKRL.realData[X] = XYZ.x * 1000.0;
            toKRL.realData[Y] = XYZ.y * 1000.0;
            toKRL.realData[Z] = XYZ.z * 1000.0;
            // WARNING: A is rotation around Z , B around Y, C around X
            toKRL.realData[A] = RPY.z * conv;
            toKRL.realData[B] = RPY.y * conv;
            toKRL.realData[C] = RPY.x * conv;

            setBit(toKRL.boolData,MASK_0,XYZ_mask.x);
            setBit(toKRL.boolData,MASK_1,XYZ_mask.y);
            setBit(toKRL.boolData,MASK_2,XYZ_mask.z);
            setBit(toKRL.boolData,MASK_3,RPY_mask.z); // A
            setBit(toKRL.boolData,MASK_4,RPY_mask.y); // B
            setBit(toKRL.boolData,MASK_5,RPY_mask.x); // C
        break;
    }
    if(ptp_input_type == CARTESIAN_IN_TOOL)
    {
        log(Error) << "Tool frame is only support for LIN_REL motions" << endlog();
        return;
    }
    toKRL.intData[CMD_INPUT_TYPE] = ptp_input_type;
    setBit(toKRL.boolData,PTP_CMD,true);
    setBit(toKRL.boolData,LIN_CMD,false);
    toKRL.intData[USE_RELATIVE] = use_rel;

    if( 0 <= vel_percent && vel_percent <= 100.0)
    {
        toKRL.realData[CMD_VEL_PERCENT] = vel_percent;
    }
    else
    {
        toKRL.realData[CMD_VEL_PERCENT] = 2.0;
        log(Error) << "Vel percent should be between [0:100.], you provided ("<<vel_percent<<"), setting to 2%."<<endlog();
    }
    setBit(toKRL.boolData,KRL_LOOP_REQUESTED,true);
}

bool KRLTool::setBase(int base_number)
{
    if( ! (0 < base_number && base_number <= 16) )
    {
        log(Error) << "Base number should be beetween [1:16]" << endlog();
        return false;
    }
    bool wait_until_done = true;
    toKRL.intData[BASE] = base_number;
    if(wait_until_done)
    {
      port_toKRL.write(toKRL);
      port_fromKRL.read(fromKRL);
      TimeService::ticks timestamp = TimeService::Instance()->getTicks();
      while(fromKRL.intData[BASE] != base_number)
      {
          if(TimeService::Instance()->secondsSince( timestamp ) > RTT::Seconds(5.0))
          {
              log(Error) << "\n\n\n Could not set Base "<<base_number<<" , please check teach pendant \n\n" << endlog();
              return false;
          }
          port_fromKRL.read(fromKRL);
          usleep(500);
          //log(Info) << "----- Waiting -----" << endlog();
      }
      toKRL.intData[BASE] = 0;
      port_toKRL.write(toKRL);
    }
  return true;
}

bool KRLTool::setTool(int tool_number)
{
    if( ! (0 < tool_number && tool_number <= 16) )
    {
        log(Error) << "Tool number should be beetween [1:16]" << endlog();
        return false;
    }

    bool wait_until_done = true;
    toKRL.intData[TOOL] = tool_number;
    //setBit(toKRL.boolData,SET_TOOL,true);
    //setBit(toKRL.boolData,KRL_LOOP_REQUESTED,true);
    if(wait_until_done)
    {
      port_toKRL.write(toKRL);
      port_fromKRL.read(fromKRL);
      TimeService::ticks timestamp = TimeService::Instance()->getTicks();
      while(fromKRL.intData[TOOL] != tool_number)
      {
          if(TimeService::Instance()->secondsSince( timestamp ) > RTT::Seconds(5.0))
          {
              log(Error) << "\n\n\n Could not set Tool "<<tool_number<<" , please check teach pendant \n\n" << endlog();
              return false;
          }
          port_fromKRL.read(fromKRL);
          usleep(500);
          //log(Info) << "----- Waiting -----" << endlog();
      }
      toKRL.intData[TOOL] = 0;
      port_toKRL.write(toKRL);
    }
    return true;
}
void KRLTool::setVELPercent(float vel_percent)
{
  if( 0 <= vel_percent && vel_percent <= 100)
  {
    toKRL.realData[OV_VEL_PERCENT] = vel_percent;
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

bool KRLTool::hasKRLReset()
{
    for(int i=0;i<FRI_USER_SIZE;i++)
    {
        if(isNoAckNeededVar(i))
            continue;
        if(getBit(fromKRL.boolData,i))
            return false;
    }
    return true;
}

void KRLTool::cancelMotion()
{
    cancelGoals();
    setBit(toKRL.boolData,CANCEL_MOTION,true);
}

void KRLTool::updateHook()
{
    static bool first_loop = true;
    static bool has_cmd = false;
    static fri_uint16_t to_krl_bool_data = 0;

    if(!first_loop)
        port_fromKRL.read(fromKRL);

    // Incoming ROS Int message
//     if(port_intDataToKRL_ros.read(intDataToKRL) == NewData)
//     {
//         for(unsigned i=0;i<FRI_USER_SIZE && i<intDataToKRL.data.size();++i)
//             if(intDataToKRL.data[i] != lwr::ROS_MASK_NO_UPDATE)
//                 toKRL.intData[i] = static_cast<fri_int32_t>(intDataToKRL.data[i]);
//         setBit(toKRL.boolData,KRL_LOOP_REQUESTED,true);
//     }
//
//     // Incoming ROS Float message
//     if(port_realDataToKRL_ros.read(realDataToKRL) == NewData)
//     {
//         for(unsigned i=0;i<FRI_USER_SIZE && i<realDataToKRL.data.size();++i)
//             if(realDataToKRL.data[i] != lwr::ROS_MASK_NO_UPDATE)
//                 toKRL.realData[i] = static_cast<fri_float_t>(realDataToKRL.data[i]);
//         setBit(toKRL.boolData,KRL_LOOP_REQUESTED,true);
//     }

    // To KRL
    if(!has_cmd && getBit(toKRL.boolData,KRL_LOOP_REQUESTED))
    {
        to_krl_bool_data = toKRL.boolData;
        has_cmd = true;
        //printAll();
    }


    if(has_cmd)
    {
        if(getBit(fromKRL.boolData,KRL_LOOP_REQUESTED))
        {

            bool ack_ptp = getBit(to_krl_bool_data,PTP_CMD)
                && ptp_current_gh.isValid()
                && ptp_current_gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE;

            bool ack_lin = getBit(to_krl_bool_data,LIN_CMD)
                && lin_current_gh.isValid()
                && lin_current_gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE;

            resetBoolToKRL();

            while(!hasKRLReset())
            {
                verifySpecialCases();
                port_toKRL.write(toKRL);
                port_fromKRL.read(fromKRL);
                usleep(250);
                // log(Info) << "----- Waiting-----" << endlog();
            }

            if(ack_ptp)
            {
                // log(Info) << "----- PTP ACKED   -----" << endlog();
                ptp_current_gh.setSucceeded(ptp_result);
            }
            if(ack_lin)
            {
                // log(Info) << "----- LIN ACKED   -----" << endlog();
                lin_current_gh.setSucceeded(lin_result);
            }


            to_krl_bool_data = 0;
            has_cmd = false;

            // log(Info) << "----- ACKED   -----" << endlog();
        }

        if(lin_current_gh.isValid()
        && lin_current_gh.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
        {
            // Feedback on LIN
            FlowStatus f = port_ft_sensor.readNewest(ft_sensor_wrench);
            double fx = ft_sensor_wrench.wrench.force.x;
            double fy = ft_sensor_wrench.wrench.force.y;
            double fz = ft_sensor_wrench.wrench.force.z;
            double tx = ft_sensor_wrench.wrench.torque.x;
            double ty = ft_sensor_wrench.wrench.torque.y;
            double tz = ft_sensor_wrench.wrench.torque.z;

            double norm_f = std::sqrt(/*fx*fx + fy*fy +*/ fz*fz);

            if(lin_current_gh.getGoal()->stop_on_force && (f == NoData || !port_ft_sensor.connected()))
            {
                log(Error) << "You asked to stop on force, but nothing seems to be connected to "<<port_ft_sensor.getName() << endlog();
            }

            if(lin_current_gh.getGoal()->stop_on_force
            && norm_f >= lin_current_gh.getGoal()->max_allowed_force)
            {
                // Stop The mouvement, we reach the max allowed force
                setBit(toKRL.boolData,CANCEL_MOTION,true);
                lin_current_gh.setAborted();
                log(Warning) << "LIN Goal was aborted because max force was reached ("<<norm_f<<" N)"<<endlog();
            }
        }
        // log(Debug) << "Waiting for ack from KRC" << endlog();
    }
    else
    {
        resetBoolToKRL();
        to_krl_bool_data = 0;
        has_cmd = false;
    }

    verifySpecialCases();

    // Writing
    port_toKRL.write(toKRL);

    // Joint Impedance Commands
    if(do_send_imp_cmd)
    {
        port_JointImpedanceCommand.write(cmd);
        do_send_imp_cmd = false;
    }

    if(first_loop)
    {
        port_fromKRL.read(fromKRL);
        first_loop = false;
    }

        /* To ROS for plotting */
    for(int i=0;i<FRI_USER_SIZE;++i)
        intDataFromKRL.data[i] = fromKRL.intData[i];
    for(int i=0;i<FRI_USER_SIZE;++i)
        realDataFromKRL.data[i] = fromKRL.realData[i];
    for(int i=0;i<FRI_USER_SIZE;++i)
        boolDataFromKRL.data[i] = getBit(fromKRL.boolData , i);

    port_intDataFromKRL_ros.write(intDataFromKRL);
    port_realDataFromKRL_ros.write(realDataFromKRL);
    port_boolDataFromKRL_ros.write(boolDataFromKRL);

}
void KRLTool::verifySpecialCases()
{
    if(getBit(fromKRL.boolData,SET_VEL))
    {
        setBit(toKRL.boolData,SET_VEL,false);
    }

    if(getBit(fromKRL.boolData,CANCEL_MOTION))
    {
        setBit(toKRL.boolData,CANCEL_MOTION,false);
    }
}
void KRLTool::addNoAckNeededVar(int special_case)
{
    bypass_ack_idx.push_back(special_case);
}

bool KRLTool::isNoAckNeededVar(int test_case)
{
    for(int i=0;i<bypass_ack_idx.size();++i)
        if(bypass_ack_idx[i] == test_case)
            return true;
    return false;
}

void KRLTool::resetBoolToKRL()
{
    for(unsigned int i=0;i<FRI_USER_SIZE;++i)
    {
        // Special cases do not need to be reset
        if(!isNoAckNeededVar(i))
            setBit(toKRL.boolData,i,false);
    }
}

void KRLTool::stopHook()
{
    toKRL.boolData = 0;
    port_toKRL.write(toKRL);
}

}
