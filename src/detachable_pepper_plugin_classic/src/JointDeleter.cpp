#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo {
class JointDeleter : public WorldPlugin {
public:
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override {
        // Initialize transport node
        node = transport::NodePtr(new transport::Node());
        node->Init(_world->Name());
        
        // Subscribe to deletion topic
        del_sub = node->Subscribe("~/delete_joint", 
            &JointDeleter::OnDeleteCommand, this);
        
        world = _world;
    }

private:
    void OnDeleteCommand(ConstGzStringPtr &_msg) {
        std::lock_guard<std::mutex> lock(mutex);
        joint_to_delete = _msg->data();
    }

    void OnPreUpdate() {
        if (!joint_to_delete.empty()) {
            physics::JointPtr joint = world->GetJoint(joint_to_delete); //use EntityByName? Joint is an Entity
            
            if (joint) {
                // Proper joint removal sequence
                joint->Detach(); //this is a real method
                // world->RemoveJoint(joint_to_delete);
                
                gzmsg << "Deleted joint: " << joint_to_delete << std::endl;
            }
            
            joint_to_delete.clear();
        }
    }

    transport::NodePtr node;
    transport::SubscriberPtr del_sub;
    physics::WorldPtr world;
    
    std::string joint_to_delete;
    std::mutex mutex;
};

GZ_REGISTER_WORLD_PLUGIN(JointDeleter)
}
