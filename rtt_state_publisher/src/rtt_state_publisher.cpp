// Antoine Hoarau hoarau.robotics@gmail.com
// Copyright ISIR 2015
#include <rtt/Component.hpp>
#include <sensor_msgs/JointState.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <Eigen/Core>
#include <time.h>
#include <rtt/os/TimeService.hpp>
#include <sstream>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/Operation.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <urdf/model.h>
#include <rtt_roscomm/rtt_rostopic.h>
#include <std_msgs/Float32.h>
#include <rtt_rosparam/rosparam.h>
#include <rtt_ros_kdl_tools/mqueue_connpolicy.hpp>

class RTTStatePublisher : public RTT::TaskContext{
public:
    RTTStatePublisher(std::string const& name):
        robot_name("lwr"),
        robot_namespace(""),
        link_prefix(""),
        RTT::TaskContext(name)
    {
            addProperty("robot_name",robot_name);
            addProperty("robot_namespace",robot_namespace);
            addProperty("link_prefix",link_prefix);
            addProperty("robot_description",robot_description);
            addPort("JointPosition", port_JointPosition).doc("");
            addPort("JointVelocity", port_JointVelocity).doc("");
            addPort("JointTorque", port_JointTorque).doc("");
            addPort("JointStates", port_JointStates).doc("");
            addPort("period", port_period).doc("");
            addOperation("connectMQueue",&RTTStatePublisher::connectMQueue,this,RTT::OwnThread);
            addOperation("connectDefault",&RTTStatePublisher::connectDefault,this,RTT::OwnThread);
    }
    bool configureHook(){
        boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
            this->getProvider<rtt_rosparam::ROSParam>("rosparam");
            
        if(rosparam)
            rosparam->getAllRelative();
        else return false;

        if(! initJointStateMsgFromString(robot_description,joint_state)) return false;

        jnt_pos.resize(joint_state.position.size());
        jnt_vel.resize(joint_state.velocity.size());
        jnt_trq.resize(joint_state.effort.size());
        jnt_pos.setZero();
        jnt_vel.setZero();
        jnt_trq.setZero();

        RTT::log(RTT::Info) << "Provided robot name : "<<robot_name <<RTT::endlog();

        port_JointStates.createStream(rtt_roscomm::topic("joint_states"));
        port_period.createStream(rtt_roscomm::topic(getName()+"/period"));

        last = rtt_rosclock::host_now();
        return true;
    }
    
    bool connectMQueue(const std::string& robot_name) // Bare/CORBA
    {
        RTT::log(RTT::Warning) << "Trying to connect to "<<robot_name<<" default joint ports using MQueue"<<RTT::endlog();
        if(hasPeer(robot_name)){
            /*RTT::TaskContext* peer = getPeer(robot_name);
            if(peer == NULL){
                RTT::log(RTT::Error) << "Peer is NULL"<<RTT::endlog();
                return false;
            }*/
            port_JointPosition.connectTo(getPeer(robot_name)->getPort("JointPosition"),RTT::MQConnPolicy::mq_data());
            port_JointVelocity.connectTo(getPeer(robot_name)->getPort("JointVelocity"),RTT::MQConnPolicy::mq_data());
            port_JointTorque.connectTo(getPeer(robot_name)->getPort("JointTorque"),RTT::MQConnPolicy::mq_data());      
        }else{
            return false;
        }     
        return port_JointPosition.connected() || port_JointVelocity.connected() || port_JointTorque.connected();
    }
    
    bool connectDefault(const std::string& robot_name) // MQueue
    {
        if(hasPeer(robot_name)){
            RTT::log(RTT::Info) << "Trying to connect to default joint ports."<<RTT::endlog();
            RTT::TaskContext* peer = getPeer(robot_name);
            if(peer == NULL) return false;
            RTT::ConnPolicy policy = RTT::ConnPolicy::data();
            
            port_JointPosition.connectTo(peer->getPort("JointPosition"),policy);
            port_JointVelocity.connectTo(peer->getPort("JointVelocity"),policy);
            port_JointTorque.connectTo(peer->getPort("JointTorque"),policy);
        }else{
            return false;
        }     
        return port_JointPosition.connected() || port_JointVelocity.connected() || port_JointTorque.connected();
    }
    
    bool initJointStateMsgFromString(const std::string& robot_description, sensor_msgs::JointState& joint_state)
    {        
        RTT::log(RTT::Debug)<<"Creating Joint State message from robot_description" << RTT::endlog();
        urdf::Model model;

        // Verify if provided robot_description is correct
        if(!model.initString(robot_description)) {
            RTT::log(RTT::Error) << "Could not init URDF." <<RTT::endlog();
            return false;
        }

        RTT::log(RTT::Debug) << "Robot name : "<<model.getName()<< RTT::endlog();
        // Create a blank joint state msg
        joint_state = sensor_msgs::JointState();

        // Reading Joints from urdf model
        for (std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator j=model.joints_.begin(); j!=model.joints_.end(); ++j)
        {
            if(j->second->limits)
            {
                if(j->second->limits->lower == j->second->limits->upper)
                {
                    // NOTE: Setting pseudo fixed-joints to FIXED, so that KDL does not considers them.
                    RTT::log(RTT::Debug) << "Removing fixed joint "<<j->second->name<<std::endl;
                    continue;
                }
            }
                
            if(j->second->type != urdf::Joint::FIXED && 
               j->second->type != urdf::Joint::FLOATING)
            {
                const std::string name = j->first;
                RTT::log(RTT::Debug)<<"Adding Joint "<< name << RTT::endlog();
                joint_state.name.push_back(name);
                joint_state.position.push_back(0.);
                joint_state.velocity.push_back(0.);
                joint_state.effort.push_back(0.);
            }
        }
        return true;
    }

    void updateHook() {       
        jnt_pos_fs = port_JointPosition.readNewest(jnt_pos);
        jnt_vel_fs = port_JointVelocity.readNewest(jnt_vel);
        jnt_trq_fs = port_JointTorque.readNewest(jnt_trq);
        
        now = rtt_rosclock::host_now();
        period = (now-last).toSec();
        period_out.data = period;
        last = now;
        
        joint_state.header.stamp = now;       
    
        Eigen::Map<Eigen::VectorXd>(joint_state.position.data(),jnt_pos.rows()) = jnt_pos;
        Eigen::Map<Eigen::VectorXd>(joint_state.velocity.data(),jnt_vel.rows()) = jnt_vel;
        Eigen::Map<Eigen::VectorXd>(joint_state.effort.data(),jnt_trq.rows()) = jnt_trq;

        if(joint_state.header.stamp.isZero() && (
            (jnt_pos_fs == RTT::NewData) || 
            (jnt_vel_fs == RTT::NewData) || 
            (jnt_trq_fs == RTT::NewData)))
        {
            RTT::log(RTT::Error)<<"rtt_rosclock::host_now() returned 0, please check if /clock is published"<<RTT::endlog();
            joint_state.header.stamp = rtt_rosclock::rtt_now();
        }
        port_JointStates.write(joint_state);
        
        port_period.write(period_out);
    }

protected:
    RTT::OutputPort<std_msgs::Float32> port_period;
    sensor_msgs::JointState joint_state;
    RTT::OutputPort<sensor_msgs::JointState> port_JointStates;
    RTT::InputPort<Eigen::VectorXd > port_JointPosition;
    RTT::InputPort<Eigen::VectorXd > port_JointVelocity;
    RTT::InputPort<Eigen::VectorXd > port_JointTorque;

    RTT::FlowStatus jnt_pos_fs,jnt_vel_fs,jnt_trq_fs;

    Eigen::VectorXd jnt_pos,jnt_vel,jnt_trq;

    ros::Time now,last;
    std::string robot_name,robot_namespace,link_prefix;
    double period;
    std::string robot_description;
    std_msgs::Float32 period_out;
};

ORO_CREATE_COMPONENT(RTTStatePublisher)
