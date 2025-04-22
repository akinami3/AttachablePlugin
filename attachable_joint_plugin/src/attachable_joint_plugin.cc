#include <gz/plugin/Register.hh> 
#include "attachable_joint_plugin.hh"

#include <iostream>
#include <gz/common/Profiler.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/Util.hh>

using namespace attachable_joint;

//////////////////////////////////////////////////
AttachableJoint::AttachableJoint()
{
  // デフォルト attachtopic はヘッダで初期化済み
}

//////////////////////////////////////////////////
void AttachableJoint::Configure(const gz::sim::Entity      &/*_entity*/,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                gz::sim::EntityComponentManager       &/*_ecm*/,
                                gz::sim::EventManager                &/*_eventMgr*/)
{
  if (_sdf->HasElement("attachtopic"))
    this->attachtopic = _sdf->Get<std::string>("attachtopic");
  else
    gzmsg << "[AttachableJoint] using default topic ["
           << this->attachtopic << "]\n";

  this->suppressChildWarning =
    _sdf->Get<bool>("suppress_child_warning",
                    this->suppressChildWarning).first;
  this->suppressParentWarning =
    _sdf->Get<bool>("suppress_parent_warning",
                    this->suppressParentWarning).first;

  this->validConfig = true;
}

//////////////////////////////////////////////////
void AttachableJoint::PreUpdate(const gz::sim::UpdateInfo     &/*_info*/,
                                gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("AttachableJoint::PreUpdate");

  if (this->not_initialized)
  {
    this->node.Subscribe(this->attachtopic,
                         &AttachableJoint::OnAttachRequest,
                         this);
    gzmsg << "[AttachableJoint] subscribed to ["
           << this->attachtopic << "]\n";

    this->error_topic =
      this->node.Advertise<gz::msgs::Int32>(
        this->attachtopic + "/error");

    this->not_initialized = false;
  }

  gz::msgs::Int32 msg;

  // --- Attach リクエスト処理 ---
  if (this->validConfig && this->attachRequested)
  {
    bool createNew = true;
    for (auto &item : this->attachableJointList)
    {
      if (item.second == this->attachableJointName)
      {
        createNew = false;
        break;
      }
    }

    if (createNew)
    {
      auto pmodel = _ecm.EntityByComponents(
        gz::sim::components::Model(),
        gz::sim::components::Name(this->parentModelName));

      if (pmodel != gz::sim::kNullEntity)
      {
        this->parentLinkEntity = _ecm.EntityByComponents(
          gz::sim::components::Link(),
          gz::sim::components::ParentEntity(pmodel),
          gz::sim::components::Name(this->parentLinkName));

        if (this->parentLinkEntity != gz::sim::kNullEntity)
        {
          auto cmodel = _ecm.EntityByComponents(
            gz::sim::components::Model(),
            gz::sim::components::Name(this->childModelName));

          if (cmodel != gz::sim::kNullEntity)
          {
            this->childLinkEntity = _ecm.EntityByComponents(
              gz::sim::components::Link(),
              gz::sim::components::ParentEntity(cmodel),
              gz::sim::components::Name(this->childLinkName));

            if (this->childLinkEntity != gz::sim::kNullEntity)
            {
              this->attachableJointEntity =
                _ecm.CreateEntity();
              _ecm.CreateComponent(
                this->attachableJointEntity,
                gz::sim::components::DetachableJoint({
                  this->parentLinkEntity,
                  this->childLinkEntity,
                  "fixed"}));

              this->attachableJointList.emplace_back(
                this->attachableJointEntity,
                this->attachableJointName);

              this->initialized     = true;
              this->attachRequested = false;
              msg.set_data(0);
            }
            else
            {
              ignwarn << "[AttachableJoint] child link ["
                      << this->childLinkName
                      << "] not found.\n";
              this->attachRequested = false;
              msg.set_data(1);
            }
          }
          else if (!this->suppressChildWarning)
          {
            ignwarn << "[AttachableJoint] child model ["
                    << this->childModelName
                    << "] not found.\n";
            this->attachRequested = false;
            msg.set_data(1);
          }
        }
        else
        {
          ignwarn << "[AttachableJoint] parent link ["
                  << this->parentLinkName
                  << "] not found.\n";
          this->attachRequested = false;
          msg.set_data(1);
        }
      }
      else if (!this->suppressParentWarning)
      {
        ignwarn << "[AttachableJoint] parent model ["
                << this->parentModelName
                << "] not found.\n";
        this->attachRequested = false;
        msg.set_data(1);
      }
    }
    else
    {
      this->attachRequested = false;
      msg.set_data(2);
    }
  }

  // --- Detach リクエスト処理 ---
  if (this->initialized && this->detachRequested)
  {
    msg.set_data(1);
    for (size_t i = 0; i < this->attachableJointList.size(); ++i)
    {
      if (this->attachableJointList[i].second
          == this->attachableJointName)
      {
        _ecm.RequestRemoveEntity(
          this->attachableJointList[i].first);
        this->attachableJointList.erase(
          this->attachableJointList.begin() + i);
        this->detachRequested = false;
        msg.set_data(0);
        break;
      }
    }
  }

  // ← ここを修正：’->’ → ‘.’ で呼び出し
  this->error_topic.Publish(msg);
}

//////////////////////////////////////////////////
void AttachableJoint::OnAttachRequest(
  const gz::msgs::StringMsg &_msg)
{
  gzmsg << "[AttachableJoint] received: "
         << _msg.data() << "\n";

  // [PM][PL][CM][CL][attach|detach]
  std::string str = _msg.data();
  std::vector<std::string> parts;
  std::size_t pos = 0;

  while (true)
  {
    auto b = str.find('[', pos);
    if (b == std::string::npos) break;
    auto e = str.find(']', b);
    if (e == std::string::npos) break;
    parts.push_back(str.substr(b+1, e-b-1));
    pos = e + 1;
  }

  if (parts.size() < 5)
  {
    ignerr << "[AttachableJoint] invalid format: "
           << _msg.data() << "\n";
    return;
  }

  this->parentModelName   = parts[0];
  this->parentLinkName    = parts[1];
  this->childModelName    = parts[2];
  this->childLinkName     = parts[3];
  std::string req         = parts[4];

  this->attachableJointName =
    this->parentModelName + "_" +
    this->parentLinkName  + "_" +
    this->childModelName  + "_" +
    this->childLinkName;

  if (req == "attach")
    this->attachRequested = true;
  else if (req == "detach" && !this->not_initialized)
    this->detachRequested = true;
}

// ここにマクロを配置（必ず .cc の末尾に！）
GZ_ADD_PLUGIN(attachable_joint::AttachableJoint,
              gz::sim::v8::System,
              attachable_joint::AttachableJoint::ISystemConfigure,
              attachable_joint::AttachableJoint::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(AttachableJoint,
                    "attachable_joint::AttachableJoint")
