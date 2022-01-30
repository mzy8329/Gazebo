#include <ignition/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{
    class Factory: public WorldPlugin
    {
        private:
        public:
            void Load(physics::WorldPtr _parent, sdf::ElementPtr)
            {
                //1. insert from file
                _parent ->InsertModelFile("model://mcar_sdf");

                //2. insert by codes
                sdf::SDF someSDFcodes;
                someSDFcodes.SetFromString(
                "\
                <?xml version='1.0'?/>\
                <sdf version='1.6'>\
                    <model name='temp'>\
                        <pose>2 2 0.5 0 0 0</pose>\
                        <link name='link0'>\
                            <visual name='visual'>\
                                <geometry>\
                                    <box>\
                                        <size>1 1 1</size>\
                                    </box>\
                                </geometry>\
                            </visual>\
                            <collision name='collision'>\
                                <geometry>\
                                    <box>\
                                        <size>1 1 1</size>\
                                    </box>\
                                </geometry>\
                            </collision>\
                        </link>\
                    </model>\
                </sdf>\
                ");
                sdf::ElementPtr model= someSDFcodes.Root()->GetElement("model");
                model->GetAttribute("name")->SetFromString("yourName");
                _parent->InsertModelSDF(someSDFcodes);

                //3. insert by topic
                transport::NodePtr node(new transport::Node());
                node->Init(_parent->Name());
                transport::PublisherPtr factoryPub = node->Advertise<msgs::Factory>("~/factory");
                

                msgs::Factory msg;
                msg.set_sdf_filename("model://camera");
                msgs::Set(msg.mutable_pose(),  ignition::math::Pose3d(ignition::math::Vector3d(-2, -2, 1),  ignition::math::Quaterniond(0, 0, 0)));

                factoryPub->Publish(msg);
            }
    };
    GZ_REGISTER_WORLD_PLUGIN(Factory);
}
