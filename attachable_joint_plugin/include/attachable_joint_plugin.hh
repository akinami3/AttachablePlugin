#ifndef ATTACHABLE_JOINT_HPP
#define ATTACHABLE_JOINT_HPP

#include <string>
#include <vector>
#include <utility>

#include <sdf/Element.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/int32.pb.h>
#include <gz/transport/Node.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/DetachableJoint.hh>

namespace attachable_joint {

class AttachableJoint :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  AttachableJoint();
  ~AttachableJoint() override = default;

  // ISystemConfigure
  void Configure(const gz::sim::Entity            &/*_entity*/,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager     &/*_ecm*/,
                 gz::sim::EventManager              &/*_eventMgr*/) override;

  // ISystemPreUpdate
  void PreUpdate(const gz::sim::UpdateInfo           &/*_info*/,
                 gz::sim::EntityComponentManager     &_ecm) override;

private:
  void OnAttachRequest(const gz::msgs::StringMsg &_msg);

  std::string attachtopic{"AttachableJoint"};
  bool suppressChildWarning{false};
  bool suppressParentWarning{false};
  bool validConfig{false};
  bool not_initialized{true};
  bool attachRequested{false};
  bool detachRequested{false};
  bool initialized{false};

  std::string parentModelName;
  std::string parentLinkName;
  std::string childModelName;
  std::string childLinkName;
  std::string attachableJointName;

  gz::sim::Entity parentLinkEntity{gz::sim::kNullEntity};
  gz::sim::Entity childLinkEntity{gz::sim::kNullEntity};
  gz::sim::Entity attachableJointEntity{gz::sim::kNullEntity};

  std::vector<std::pair<gz::sim::Entity, std::string>> attachableJointList;

  gz::transport::Node      node;
  gz::transport::Node::Publisher error_topic;
};

}  // namespace attachable_joint

#endif  // ATTACHABLE_JOINT_HPP
