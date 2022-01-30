#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
    class ModelPush: public ModelPlugin
    {
        private:
            physics::ModelPtr model;
            event::ConnectionPtr updateConnection;

        public:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr  )
            {
                this->model = _parent;
                this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelPush::OnUpdate, this));
            }

            void OnUpdate()
            {
                this->model->SetLinearVel(ignition::math::Vector3d(1, 0, 0));
            }
    };
    GZ_REGISTER_MODEL_PLUGIN(ModelPush);
} // namespace gazebo
