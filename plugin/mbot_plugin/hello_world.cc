#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
    class hello_worldPlugin: public WorldPlugin
    {
        public:
            hello_worldPlugin():WorldPlugin()
            {
                std::cout<<"hello world"<<std::endl;
            }

            void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
            {
                ;;
            }
    };
    GZ_REGISTER_WORLD_PLUGIN(hello_worldPlugin);
}
