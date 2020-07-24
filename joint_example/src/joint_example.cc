#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <string_view>
#include <gazebo/physics/Collision.hh>

namespace gazebo
{
  class WorldUuvPlugin : public WorldPlugin
  {

  enum states{
    unconnectable_unlocked, connectable_unlocked, connectable_locked
  };

  private: physics::WorldPtr world;

  private: physics::ModelPtr blockAModel;

  private: physics::ModelPtr blockBModel;

  private: physics::LinkPtr blockALink;

  private: physics::LinkPtr blockBLink;

  private: bool isJointInitiated = false;

  private: physics::JointPtr prismaticJoint;

  private: gazebo::event::ConnectionPtr updateConnection;


  public: WorldUuvPlugin() : WorldPlugin(){}

  public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
      this->world = _world;
      this->blockAModel = this->world->ModelByName("blockA");
      this->blockBModel = this->world->ModelByName("blockB");

      this->blockALink = this->blockAModel->GetLink("blockA");
      this->blockBLink   = this->blockBModel->GetLink("blockB");

      this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
          std::bind(&WorldUuvPlugin::Update, this));
    }

  public: void Update()
    {
      // connect the socket and the plug after 5 seconds
      if (this->world->SimTime() > 20.0 && this->isJointInitiated == false)
      {
        this->isJointInitiated = true;
        this->prismaticJoint = blockBModel->CreateJoint(
          "blockA_blockB_joint",
          "prismatic",
          blockALink,
          blockBLink);

        prismaticJoint->Load(this->blockALink, this->blockBLink, 
          ignition::math::Pose3<double>(ignition::math::Vector3<double>(1, 0, 0), 
          ignition::math::Quaternion<double>(0, 0, 0, 0)));

        prismaticJoint->SetUpperLimit(0, 0.3);
        prismaticJoint->Init();
        prismaticJoint->SetAxis(0, ignition::math::Vector3<double>(1, 0, 0));
        ignition::math::Vector3d prismatixAxis = prismaticJoint->LocalAxis(0);

        printf("%.2f %.2f %.2f   \n", prismatixAxis[0], prismatixAxis[1], prismatixAxis[2]);
        printf("should be 1 0 0   \n");
      }

      if (this->isJointInitiated){
        blockBLink->SetSelfCollide(true);
        prismaticJoint->SetParam("friction", 0, .05);
        blockBLink->AddForce(ignition::math::Vector3<double>(-120, 0, 0));
      }
      bool const displayForce = false;
      if (displayForce){
        // Two problems with this
        // 1. block A feels force before anything touches it
        // 2. Where is the force of gravity?
        // 3. During the collision, no spike in the force in the X direction
       ignition::math::Vector3d forceProbe = blockALink->RelativeForce();
        printf("%.2f %.2f %.2f   \n", abs(forceProbe[0]), abs(forceProbe[1]), abs(forceProbe[2]));
      }
    }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldUuvPlugin)
} // namespace gazebo
