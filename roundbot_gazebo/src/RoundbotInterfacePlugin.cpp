/*******************************************************************************
BSD 2-Clause License

Copyright (c) 2025, Micho Radovnikovich

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <roundbot_gazebo/RoundbotInterfacePlugin.hpp>

#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>

using namespace gz;
using namespace gz::sim;
using namespace systems;

namespace roundbot_gazebo {

math::Pose3d worldPose(const Entity &_entity, const EntityComponentManager &_ecm)
{
  auto pose_comp = _ecm.Component<components::Pose>(_entity);
  if (pose_comp == nullptr) {
    gzwarn << "Trying to get world pose from entity [" << _entity << "], which doesn't have a pose component" << std::endl;
    return math::Pose3d();
  }

  // work out pose in world frame
  math::Pose3d pose = pose_comp->Data();
  auto p = _ecm.Component<components::ParentEntity>(_entity);
  while (p) {
    // get pose of parent entity
    auto parentPose = _ecm.Component<components::Pose>(p->Data());
    if (!parentPose) break;
    // transform pose
    pose = parentPose->Data() * pose;
    // keep going up the tree
    p = _ecm.Component<components::ParentEntity>(p->Data());
  }
  return pose;
}

RoundbotInterfacePlugin::RoundbotInterfacePlugin() {}
RoundbotInterfacePlugin::~RoundbotInterfacePlugin() {}

void RoundbotInterfacePlugin::Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm, EventManager & /*_eventMgr*/)
{
  this->model_ = Model(_entity);
  if (!this->model_.Valid(_ecm)) {
    gzerr << "Roundbot system plugin should be attached to a model" << " entity. Failed to initialize." << std::endl;
    return;
  }

  this->publish_ground_truth_pose_ = _sdf->Get<bool>("pub_tf", true).first;

  this->left_joint_ = this->model_.JointByName(_ecm, "left_wheel_joint");
  this->right_joint_ = this->model_.JointByName(_ecm, "right_wheel_joint");
  this->left_speed_cmd_ = 0.0;
  this->right_speed_cmd_ = 0.0;

  this->left_motor_ = std::make_shared<roundbot_gazebo::MotorSim>(MAX_MOTOR_SPEED, 8.0, 8.0);
  this->right_motor_ = std::make_shared<roundbot_gazebo::MotorSim>(MAX_MOTOR_SPEED, 8.0, 8.0);

  this->node_.Subscribe("/model/" + model_.Name(_ecm) + "/left_speed_cmd", &RoundbotInterfacePlugin::recvLeftCmd, this);
  this->node_.Subscribe("/model/" + model_.Name(_ecm) + "/right_speed_cmd", &RoundbotInterfacePlugin::recvRightCmd, this);
  this->pub_twist_ = this->node_.Advertise<msgs::Twist>("/model/" + model_.Name(_ecm) + "/twist");

  if (this->publish_ground_truth_pose_) {
    this->pub_pose_ = this->node_.Advertise<msgs::Pose_V>("/model/" + model_.Name(_ecm) + "/pose");
  }
}

void RoundbotInterfacePlugin::recvLeftCmd(const gz::msgs::Double &msg) { this->left_speed_cmd_ = msg.data(); }

void RoundbotInterfacePlugin::recvRightCmd(const gz::msgs::Double &msg) { this->right_speed_cmd_ = msg.data(); }

void RoundbotInterfacePlugin::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  this->current_time_ = _info.realTime.count();
  if (_info.dt.count() == 0) {
    return;
  }

  double dt = 1e-9 * _info.dt.count();
  double left_actual = left_motor_->iterate(left_speed_cmd_, dt);
  double right_actual = right_motor_->iterate(right_speed_cmd_, dt);
  _ecm.SetComponentData<components::JointVelocityCmd>(this->right_joint_, {right_actual});
  _ecm.SetComponentData<components::JointVelocityCmd>(this->left_joint_, {left_actual});
}

void RoundbotInterfacePlugin::PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm)
{
  double dt = 1e-9 * _info.dt.count();
  const math::Pose3d vehicle_pose = worldPose(this->model_.Entity(), _ecm);
  if (first_update_) {
    first_update_ = false;
    this->last_vehicle_pose_ = vehicle_pose;
    return;
  }

  double dx = vehicle_pose.Pos().X() - this->last_vehicle_pose_.Pos().X();
  double dy = vehicle_pose.Pos().Y() - this->last_vehicle_pose_.Pos().Y();
  double yaw = vehicle_pose.Rot().Yaw();
  double dyaw = yaw - this->last_vehicle_pose_.Rot().Yaw();

  double xvel = (dx * cosf(yaw) + dy * sinf(yaw)) / dt;
  double yvel = (-dx * sinf(yaw) + dy * cosf(yaw)) / dt;
  double yawvel = dyaw / dt;
  this->last_vehicle_pose_ = vehicle_pose;

  if ((1e-9 * (this->current_time_ - this->twist_pub_stamp_)) > TWIST_SAMPLE_TIME) {
    this->twist_pub_stamp_ = this->current_time_;
    msgs::Twist twist_msg;
    twist_msg.mutable_header()->mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));
    twist_msg.mutable_linear()->set_x(xvel);
    twist_msg.mutable_linear()->set_y(yvel);
    twist_msg.mutable_linear()->set_z(0.0);
    twist_msg.mutable_angular()->set_x(0.0);
    twist_msg.mutable_angular()->set_y(0.0);
    twist_msg.mutable_angular()->set_z(yawvel);
    this->pub_twist_.Publish(twist_msg);
    if (this->publish_ground_truth_pose_) {
      msgs::Pose *pose_msg = nullptr;
      this->posev_msg_.Clear();
      pose_msg = this->posev_msg_.add_pose();
      GZ_ASSERT(pose_msg != nullptr, "Pose msg is null");

      auto header = pose_msg->mutable_header();
      header->mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));
      auto frame = header->add_data();
      frame->set_key("frame_id");
      frame->add_value("world");
      auto child_frame = header->add_data();
      child_frame->set_key("child_frame_id");
      child_frame->add_value("base_footprint");
      pose_msg->set_name("world_to_footprint");
      msgs::Set(pose_msg, vehicle_pose);
      this->pub_pose_.Publish(this->posev_msg_);
    }
  }
}
}  // namespace roundbot_gazebo

// Register plugin
GZ_ADD_PLUGIN(roundbot_gazebo::RoundbotInterfacePlugin, System, roundbot_gazebo::RoundbotInterfacePlugin::ISystemConfigure, roundbot_gazebo::RoundbotInterfacePlugin::ISystemPreUpdate,
              roundbot_gazebo::RoundbotInterfacePlugin::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(roundbot_gazebo::RoundbotInterfacePlugin, "gz::sim::systems::RoundbotInterfacePlugin")
