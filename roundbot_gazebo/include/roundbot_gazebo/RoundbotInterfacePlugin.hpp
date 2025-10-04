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

#pragma once

#include <gz/msgs.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <roundbot_gazebo/MotorSim.hpp>

using namespace gz;
using namespace sim;

namespace roundbot_gazebo {

class RoundbotInterfacePlugin : public System, public ISystemConfigure, public ISystemPreUpdate, public ISystemPostUpdate
{
  public:
    RoundbotInterfacePlugin();
    ~RoundbotInterfacePlugin() override;

    void Configure(const Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf, EntityComponentManager& _ecm, EventManager& _eventMgr) override;

    void PreUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm) override;

    void PostUpdate(const UpdateInfo& _info, const EntityComponentManager& _ecm) override;

  private:
    Model model_;
    Entity left_joint_;
    Entity right_joint_;
    transport::Node node_;
    transport::Node::Publisher pub_twist_;
    transport::Node::Publisher pub_pose_;

    void recvLeftCmd(const gz::msgs::Double& msg);
    void recvRightCmd(const gz::msgs::Double& msg);

    double left_speed_cmd_;
    double right_speed_cmd_;
    MotorSim::SharedPtr left_motor_;
    MotorSim::SharedPtr right_motor_;
    math::Pose3d last_vehicle_pose_;
    bool first_update_ = true;
    uint64_t twist_pub_stamp_ = 0;
    uint64_t current_time_ = 0;
    msgs::Pose_V posev_msg_;
    bool publish_ground_truth_pose_;
    static constexpr double TWIST_SAMPLE_TIME = 0.01;
    static constexpr double MAX_MOTOR_SPEED = 17.0;
};

}  // namespace roundbot_gazebo