// Copyright ISIR 2015
// Author: Antoine Hoarau <hoarau.robotics@gmail.com>


#include <rtt_dynamics_analysis/rtt_dynamics_analysis.hpp>

KDL::Chain KukaLWR_DHnew(){
    using namespace KDL;
    Chain kukaLWR_DHnew;

    //joint 0
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::None),
                                     Frame::DH_Craig1989(0.0, 0.0, 0.31, 0.0)
                                     ));
    //joint 1
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0),
                                     Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
                                                                                                              Vector::Zero(),
                                                                                                              RotationalInertia(0.0,0.0,0.0115343,0.0,0.0,0.0))));

    //joint 2
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::DH_Craig1989(0.0, -1.5707963, 0.4, 0.0),
                                     Frame::DH_Craig1989(0.0, -1.5707963, 0.4, 0.0).Inverse()*RigidBodyInertia(2,
                                                                                                               Vector(0.0,-0.3120511,-0.0038871),
                                                                                                               RotationalInertia(-0.5471572,-0.0000302,-0.5423253,0.0,0.0,0.0018828))));

    //joint 3
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
                                     Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
                                                                                                               Vector(0.0,-0.0015515,0.0),
                                                                                                               RotationalInertia(0.0063507,0.0,0.0107804,0.0,0.0,-0.0005147))));

    //joint 4
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::DH_Craig1989(0.0, 1.5707963, 0.39, 0.0),
                                     Frame::DH_Craig1989(0.0, 1.5707963, 0.39, 0.0).Inverse()*RigidBodyInertia(2,
                                                                                                               Vector(0.0,0.5216809,0.0),
                                                                                                               RotationalInertia(-1.0436952,0.0,-1.0392780,0.0,0.0,0.0005324))));

    //joint 5
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0),
                                     Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
                                                                                                              Vector(0.0,0.0119891,0.0),
                                                                                                              RotationalInertia(0.0036654,0.0,0.0060429,0.0,0.0,0.0004226))));

    //joint 6
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
                                     Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
                                                                                                               Vector(0.0,0.0080787,0.0),
                                                                                                               RotationalInertia(0.0010431,0.0,0.0036376,0.0,0.0,0.0000101))));
    //joint 7
    kukaLWR_DHnew.addSegment(Segment(Joint(Joint::RotZ),
                                     Frame::Identity(),
                                     RigidBodyInertia(2,
                                                      Vector::Zero(),
                                                      RotationalInertia(0.000001,0.0,0.0001203,0.0,0.0,0.0))));
    return kukaLWR_DHnew;
}

bool DynamicsAnalysis::configureHook()
{
    // Get the rosparam service requester
    boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
            this->getProvider<rtt_rosparam::ROSParam>("rosparam");

    if(!rosparam) {
        RTT::log(RTT::Error) << "Could not load rosparam service." <<RTT::endlog();
        return false;
    }

    rosparam->getPrivate("robot_name");
    rosparam->getPrivate("root_link");
    rosparam->getPrivate("tip_link");

    RTT::log(RTT::Info)<<"root_link : "<<root_link<<RTT::endlog();
    RTT::log(RTT::Info)<<"tip_link : "<<tip_link<<RTT::endlog();
    
    KDL::Vector gravity_vector(0.,0.,-9.81289);
    
    if(!rtt_ros_kdl_tools::initChainFromROSParamURDF(this,robot_name,root_link,tip_link,tree,chain))
    {
        RTT::log(RTT::Error) << "Error while loading the URDF with params : "<<robot_name<<" "<<root_link<<" "<<tip_link <<RTT::endlog();
        return false;
    }
    
    if(use_robot_description == false)
    {
        RTT::log(RTT::Warning) << "NOT using robot description, using LWR KDL instead"<<RTT::endlog();
        chain = KukaLWR_DHnew();
    }
        
    ik_solver_vel.reset(new KDL::ChainIkSolverVel_pinv_nso(chain));
    id_dyn_solver.reset(new KDL::ChainDynParam(chain,gravity_vector));
    id_rne_solver.reset(new KDL::ChainIdSolver_RNE(chain,gravity_vector));
    fk_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(chain));
            
    // Store the number of degrees of freedom of the chain
    n_joints_ = chain.getNrOfJoints();

    RTT::log(RTT::Debug)<<"LWR njoints : "<<n_joints_<<" Segments : "<<chain.getNrOfSegments()<<RTT::endlog();
    
    rtt_ros_kdl_tools::initJointStateFromKDLCHain(chain,js_dyn);
    rtt_ros_kdl_tools::initJointStateFromKDLCHain(chain,js_dyn_param);
    rtt_ros_kdl_tools::initJointStateFromKDLCHain(chain,js);
    
    jnt_pos.resize(n_joints_);
    jnt_pos_cmd.resize(n_joints_);
    jnt_pos_old.resize(n_joints_);
    jnt_vel.resize(n_joints_);
    jnt_vel_old.resize(n_joints_);
    jnt_vel_cmd.resize(n_joints_);
    jnt_trq.resize(n_joints_);
    jnt_trq_cmd.resize(n_joints_);
    jnt_acc.resize(n_joints_);
    
    jnt_pos.setZero();
    jnt_pos_cmd.setZero();
    jnt_vel.setZero();
    jnt_vel_old.setZero();
    jnt_acc.setZero();
    jnt_vel_cmd.setZero();
    jnt_pos_old.setZero();
    jnt_trq.setZero();
    
    q.resize(n_joints_);
    f_ext.resize(chain.getNrOfSegments());
    gravity.resize(n_joints_);
    qdot.resize(n_joints_);
    qddot.resize(n_joints_);
    qddot.data.setZero();
    jnt_trq_kdl.resize(n_joints_);
    coriolis.resize(n_joints_);
    mass.resize(n_joints_);
    inertia.resize(n_joints_);
    
    //port_JointPosition.getDataSample();
    //port_JointVelocity.getDataSample();
    
    port_JSDyn.setDataSample(js_dyn);
    port_JSDynParam.setDataSample(js_dyn_param);
    port_JS.setDataSample(js);
    
    port_JS.createStream(rtt_roscomm::topic("/"+this->getName()+"/js"));
    port_JSDyn.createStream(rtt_roscomm::topic("/"+this->getName()+"/js_dynamics"));
    port_JSDynParam.createStream(rtt_roscomm::topic("/"+this->getName()+"/js_dynamics_params"));
    
    amplitude = 60*M_PI/180.0;
    omega = 0.35;
    phi = 0.0;
    
    kp = 500.0;
    kd = 24.0;
    kg = 1.0;
    
    for(unsigned int j=0;j<chain.getNrOfSegments();++j)
        f_ext[j] = KDL::Wrench::Zero();
    
    return true;
}
DynamicsAnalysis::DynamicsAnalysis(const std::string& name): 
TaskContext(name),
cnt(0),
root_link("lwr/link_0"),
tip_link("lwr/link_7"),
use_robot_description(true)
{
    this->ports()->addPort("JointStateDynamics",port_JSDyn).doc("Full KDL Dynamics");
    this->ports()->addPort("JointStateParam",port_JSDynParam).doc("Inertia, Coriolis and gravity terms");
    
    this->ports()->addPort("JointPosition",port_JointPosition).doc("Joint position Eigen::VectorXd");
    this->ports()->addPort("JointVelocity",port_JointVelocity).doc("Joint velocity Eigen::VectorXd");
    //this->ports()->addPort("JointTorque",port_JointTorque).doc("Joint Torque Eigen::VectorXd");
    
    this->ports()->addPort("JointTorqueCommand",port_JointTorqueCommand).doc("Joint Torque Cmd Eigen::VectorXd");
    
    this->addProperty("robot_description",robot_description).doc("");
    this->addProperty("root_link", root_link).doc("");
    this->addProperty("tip_link", tip_link).doc("");
    this->addProperty("n_joints",n_joints_).doc("");
    this->addProperty("robot_name",robot_name).doc("The name of the rtt component robot (lwr or lwr_sim)");
    
    this->addProperty("amplitude",amplitude).doc("");
    this->addProperty("omega",omega).doc("");
    this->addProperty("phi",phi).doc("");
    this->addProperty("kp",kp).doc("");
    this->addProperty("kd",kd).doc("");
    this->addProperty("kg",kg).doc("");
}
void DynamicsAnalysis::updateHook()
{
    if(port_JointPosition.readNewest(jnt_pos) == RTT::NewData)
    {        
        q.data = jnt_pos;
        
        qdot.data.setZero();
        
        if(port_JointVelocity.readNewest(jnt_vel)  == RTT::NoData)
            jnt_vel = (jnt_pos - jnt_pos_old)/static_cast<double>(getPeriod()) ;

        qdot.data = jnt_vel;
        
        jnt_acc = (jnt_vel - jnt_vel_old)/static_cast<double>(getPeriod()) ;
        
        jnt_vel_old = jnt_vel;
        jnt_pos_old = jnt_pos;
        
        id_dyn_solver->JntToCoriolis(q,qdot,coriolis);
        id_dyn_solver->JntToGravity(q,gravity);
        id_dyn_solver->JntToMass(q,mass);
        
        for(unsigned i=0;i<n_joints_;++i)
        {
            jnt_pos_cmd[i] = amplitude*sin(omega*(cnt++)*static_cast<double>(getPeriod()) + phi);
            qddot.data[i] = kp*(jnt_pos_cmd[i] - jnt_pos[i])
                            + kd*(jnt_vel_cmd[i] - jnt_vel[i]);
        }
        
        inertia = mass.data * jnt_acc;
        //js_dyn_param.position;
        Eigen::Map<Eigen::VectorXd>(js_dyn_param.position.data(),n_joints_) = inertia;
        Eigen::Map<Eigen::VectorXd>(js_dyn_param.velocity.data(),n_joints_) = coriolis.data;
        Eigen::Map<Eigen::VectorXd>(js_dyn_param.effort.data(),n_joints_) = gravity.data;
        
        int ret = id_rne_solver->CartToJnt(q,qdot,qddot,f_ext,jnt_trq_kdl);
        
        if(ret<0)
            RTT::log(RTT::Error)<<"ERROR on ret : "<<ret<<RTT::endlog();
    
        for(unsigned i=0;i<n_joints_;++i)
            jnt_trq_kdl.data[i] -= kg*gravity.data[i];
        
        Eigen::Map<Eigen::VectorXd>(js.position.data(),n_joints_) = jnt_pos;
        Eigen::Map<Eigen::VectorXd>(js.velocity.data(),n_joints_) = qdot.data;
        Eigen::Map<Eigen::VectorXd>(js.effort.data(),n_joints_) = jnt_acc;
               
        Eigen::Map<Eigen::VectorXd>(js_dyn.position.data(),n_joints_) = jnt_pos_cmd;
        Eigen::Map<Eigen::VectorXd>(js_dyn.velocity.data(),n_joints_) = jnt_vel_cmd;
        Eigen::Map<Eigen::VectorXd>(js_dyn.effort.data(),n_joints_) = jnt_trq_kdl.data;
        
        
        js_dyn.header.stamp =
        js_dyn_param.header.stamp = 
        js.header.stamp =  rtt_rosclock::host_now();
        
        port_JSDynParam.write(js_dyn_param);
        port_JSDyn.write(js_dyn);
        port_JS.write(js);
        port_JointTorqueCommand.write(jnt_trq_kdl.data);
        
    }
}
