#ifndef _SDF_ROS_CONTROL_PLUGIN_HH_
#define _SDF_ROS_CONTROL_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32.h>
#include <math.h>

namespace gazebo
{
    class Cfg
    {
    public:
        int ctrlType;
        double initPos;
        double initVel;
    };

    class SdfRosControlPlugin : public ModelPlugin
    {
    public:

        SdfRosControlPlugin()
        {
            std::cout << "Loading sdf_ros_control_plugin" << std::endl;
        }

        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // check SDF
            sdf::ElementPtr cursor = _sdf->GetElementImpl("ctrl_joint");
            if(cursor == nullptr)
            {
                std::cout << "Invalid joint count, ros control plugin not loaded\n";
                return;
            }

            this->model = _model;

            // Create gz node
            this->node = transport::NodePtr(new transport::Node());
            this->node->Init(this->model->GetWorld()->Name());

            // Initialize ros, if it has not already been initialized.
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = nullptr;
                ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
            }

            // Create our ROS node. This acts in a similar manner to the Gazebo node
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            int ctrlJointCount = 0;
            while(cursor != nullptr)
            {
                // joints to be controlled
                sdf::ParamPtr PPtr = cursor->GetAttribute("name");
                physics::JointPtr JPtr = this->model->GetJoint(this->model->GetName() + "::" + PPtr->GetAsString());
                this->ctrlJoint.emplace_back(JPtr);

                // ctrlCfg, type=1: pos, type=2: vel
                Cfg cfg;
                cfg.ctrlType = cursor->Get<int>("ctrl_type");
                cfg.initPos = cursor->Get<double>("init_pos");
                cfg.initVel = cursor->Get<double>("init_vel");
                this->ctrlCfg.emplace_back(cfg);

                // PIDParam
                auto p = cursor->Get<double>("Kp");
                auto i = cursor->Get<double>("Ki");
                auto d = cursor->Get<double>("Kd");
                common::PID param = common::PID(p, i, d);
                this->PIDParam.emplace_back(param);

                // setup controllers
                if(cfg.ctrlType == 1)
                {
                    this->model->GetJointController()->SetPositionPID(JPtr->GetScopedName(), param);
                    this->model->GetJointController()->SetPositionTarget(JPtr->GetScopedName(), cfg.initPos);
                }
                if(cfg.ctrlType == 2)
                {
                    this->model->GetJointController()->SetVelocityPID(JPtr->GetScopedName(), param);
                    this->model->GetJointController()->SetVelocityTarget(JPtr->GetScopedName(), cfg.initVel);
                }

                std::string modelName;
                auto modelNameOverride = cursor->Get<std::string>("modelNameOverride");
                if(!modelNameOverride.empty())
                {
                    modelName = modelNameOverride;
                }
                else
                {
                    modelName = this->model->GetName();
                }

                // Create a gz topic name and subscribe to it
                std::string gzTopicName;
                if(cfg.ctrlType == 1)
                {
                    gzTopicName = "~/" + modelName + "/" + PPtr->GetAsString() + "/pos_cmd";
                }
                if(cfg.ctrlType == 2)
                {
                    gzTopicName = "~/" + modelName + "/" + PPtr->GetAsString() + "/vel_cmd";
                }
                this->gzSub.emplace_back(this->node->Subscribe(gzTopicName, &SdfRosControlPlugin::OnMsg, this));

                // Create a named topic, and subscribe to it.
                std::string rosTopicName;
                if(cfg.ctrlType == 1)
                {
                    rosTopicName = "/" + modelName + "/" + PPtr->GetAsString() + "/pos_cmd";
                }
                if(cfg.ctrlType == 2)
                {
                    rosTopicName = "/" + modelName + "/" + PPtr->GetAsString() + "/vel_cmd";
                }
                ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(
                        rosTopicName,
                        1,
                        boost::bind(&SdfRosControlPlugin::OnRosMsg, this, _1, JPtr, cfg.ctrlType),
                        ros::VoidPtr(), &this->rosQueue);
                this->rosSub.emplace_back(this->rosNode->subscribe(so));

                cursor = cursor->GetNextElement("ctrl_joint");
                ctrlJointCount += 1;
            }

            std::cout << "number of joints to be controlled: " << ctrlJointCount << std::endl;
            std::cout << "-------------------------------------------------------------" << std::endl;
            for(int i = 0; i < ctrlJointCount; i++)
            {
                std::cout << this->ctrlJoint[i]->GetName() << std::endl;
                std::cout << "intiPos=" << this->ctrlCfg[i].initPos;
                std::cout << " initVel=" << this->ctrlCfg[i].initVel;
                std::cout << " ctrlType=" << this->ctrlCfg[i].ctrlType << std::endl;
                std::cout << "P=" << this->PIDParam[i].GetPGain();
                std::cout << " I=" << this->PIDParam[i].GetIGain();
                std::cout << " D=" << this->PIDParam[i].GetDGain() << std::endl;
                std::cout << "gzTopic=" << gzSub[i]->GetTopic() << std::endl;
                std::cout << "rosTopic=" << rosSub[i].getTopic() << std::endl;
                std::cout << "-------------------------------------------------------------" << std::endl;
            }

            // Spin up the queue helper thread.
            this->rosQueueThread = std::thread(std::bind(&SdfRosControlPlugin::QueueThread, this));
        }

        void QueueThread()
        {
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        void OnRosMsg(const std_msgs::Float32ConstPtr &_msg, const physics::JointPtr& joint, int ctrlType)
        {
            double setpoint = _msg->data;
            if(ctrlType == 1)
            {
                double current = this->model->GetJointController()->GetPositions().at(joint->GetScopedName());
                while(setpoint - current > M_PI)
                {
                    setpoint -= 2 * M_PI;
                }
                while(setpoint - current < -M_PI)
                {
                    setpoint += 2 * M_PI;
                }          
                this->model->GetJointController()->SetPositionTarget(joint->GetScopedName(), setpoint);
            }
            if(ctrlType == 2)
            {
                this->model->GetJointController()->SetVelocityTarget(joint->GetScopedName(), setpoint);
            }
        }

        void OnMsg(ConstVector3dPtr &_msg)
        {

        }


    private:

        transport::NodePtr node;

        std::vector<transport::SubscriberPtr> gzSub;

        physics::ModelPtr model;

        std::vector<physics::JointPtr> ctrlJoint;

        std::vector<common::PID> PIDParam;
        std::vector<Cfg> ctrlCfg;

        std::unique_ptr<ros::NodeHandle> rosNode;

        std::vector<ros::Subscriber> rosSub;

        ros::CallbackQueue rosQueue;

        std::thread rosQueueThread;
    };

    GZ_REGISTER_MODEL_PLUGIN(SdfRosControlPlugin)
}
#endif