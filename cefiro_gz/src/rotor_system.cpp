#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace cefiro::gz_plugins {

class RotorSystem final : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate {
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    const gz::sim::Model model(entity);
    body_ = model.LinkByName(ecm, "drone_body_link");
    for (int i = 0; i < 4; ++i) {
      joints_[i] = model.JointByName(ecm, "motor_" + std::to_string(i + 1));
      const std::string key = "rotor_" + std::to_string(i + 1) + "_xyz";
      if (sdf->HasElement(key)) {
        rotor_positions_[static_cast<std::size_t>(i)] =
            sdf->Get<gz::math::Vector3d>(key);
      }
    }
    if (sdf->HasElement("visual_slowdown")) {
      visual_slowdown_ = sdf->Get<double>("visual_slowdown");
    }
    if (sdf->HasElement("command_timeout_s")) {
      timeout_s_ = sdf->Get<double>("command_timeout_s");
    }
    valid_ = body_ != gz::sim::kNullEntity &&
             std::isfinite(visual_slowdown_) && visual_slowdown_ >= 1.0 &&
             std::isfinite(timeout_s_) && timeout_s_ > 0.0;
    for (const auto j : joints_) valid_ &= (j != gz::sim::kNullEntity);
    if (!valid_) {
      gzerr << "[CEFIRO] RotorSystem: invalid joints, body, or parameters\n";
      return;
    }
    for (int i = 0; i < 4; ++i) {
      const auto index = static_cast<std::size_t>(i);
      const auto topic = "/cefiro/gz/motor_" + std::to_string(i + 1);
      transport_.Subscribe<gz::msgs::Vector3d>(topic,
        [this, index](const gz::msgs::Vector3d &msg) {
          if (!std::isfinite(msg.x()) || !std::isfinite(msg.y()) ||
              !std::isfinite(msg.z())) return;
          std::lock_guard<std::mutex> lock(mutex_);
          channels_[index] = {std::max(0.0, msg.x()),
                              std::max(0.0, msg.y()), msg.z(),
                              std::chrono::steady_clock::now(), true};
        });
    }
    gzmsg << "[CEFIRO] RotorSystem: 4 motor channels ready, "
          << "speed reduction=" << visual_slowdown_ << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (!valid_ || info.paused) return;
    std::array<Channel, 4> snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      snapshot = channels_;
    }
    const auto now = std::chrono::steady_clock::now();
    auto body_pose = gz::sim::worldPose(body_, ecm);
    const auto rotation = body_pose.Rot();
    gz::math::Vector3d f_body{0, 0, 0};
    gz::math::Vector3d tau_body{0, 0, 0};
    for (std::size_t i = 0; i < 4; ++i) {
      auto c = snapshot[i];
      if (!c.received ||
          std::chrono::duration<double>(now - c.last).count() > timeout_s_) {
        c.speed = c.thrust = c.reaction = 0.0;
      }
      // Positive joint velocity follows each joint's URDF axis; CW axes
      // are -Z and CCW axes are +Z in cefiro_params.yaml.
      const std::vector<double> speed_cmd{c.speed / visual_slowdown_};
      auto *cmd = ecm.Component<gz::sim::components::JointVelocityCmd>(joints_[i]);
      if (cmd) {
        cmd->SetData(speed_cmd,
          [](const auto &, const auto &) { return false; });
      } else {
        ecm.CreateComponent(joints_[i],
          gz::sim::components::JointVelocityCmd(speed_cmd));
      }
      const gz::math::Vector3d local_force(0, 0, c.thrust);
      f_body += local_force;
      // Rotor positions in the drone_body_link frame (metres).
      // Reaction torque is already signed by the actuator model.
      tau_body += rotor_positions_[i].Cross(local_force);
      tau_body.Z() += c.reaction;
    }
    // Apply the total wrench in the WORLD frame to the drone body link.
    // Its physical constraint to Nenya transmits loads to the gimbal.
    gz::sim::Link(body_).AddWorldWrench(ecm,
      rotation.RotateVector(f_body), rotation.RotateVector(tau_body));
  }
private:
  struct Channel {
    double speed{0.0};
    double thrust{0.0};
    double reaction{0.0};
    std::chrono::steady_clock::time_point last{};
    bool received{false};
  };
  // Positions are generated from cefiro_params.yaml by Xacro.
  std::array<gz::math::Vector3d, 4> rotor_positions_{};
  std::array<gz::sim::Entity, 4> joints_{};
  gz::sim::Entity body_{gz::sim::kNullEntity};
  std::array<Channel, 4> channels_{};
  std::mutex mutex_;
  gz::transport::Node transport_;
  double visual_slowdown_{10.0};
  double timeout_s_{0.35};
  bool valid_{false};
};
}  // namespace cefiro::gz_plugins

GZ_ADD_PLUGIN(cefiro::gz_plugins::RotorSystem,
  gz::sim::System,
  cefiro::gz_plugins::RotorSystem::ISystemConfigure,
  cefiro::gz_plugins::RotorSystem::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(cefiro::gz_plugins::RotorSystem,
                    "cefiro::gz_plugins::RotorSystem")
