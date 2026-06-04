#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <ignition/math/Pose3.hh>

namespace dynamic_nav_gazebo_plugins
{
class ScriptedActorPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->model_ = std::move(model);
    this->world_ = this->model_->GetWorld();
    this->speed_ = this->ReadDouble(sdf, "speed", 0.8);
    this->z_ = this->ReadDouble(sdf, "z", 0.8);
    this->swayAmplitude_ = this->ReadDouble(sdf, "sway_amplitude", 0.06);
    this->swayRate_ = this->ReadDouble(sdf, "sway_rate", 1.0);
    this->swayPhase_ = this->ReadDouble(sdf, "sway_phase", 0.0);
    this->bobAmplitude_ = this->ReadDouble(sdf, "bob_amplitude", 0.0);
    this->speedPhase_ = this->ReadDouble(sdf, "speed_phase", 0.0);
    this->avoidRadius_ = this->ReadDouble(sdf, "avoid_radius", 0.55);
    this->laneOffset_ = this->ReadDouble(sdf, "lane_offset", 0.0);
    this->robotAvoidRadius_ = this->ReadDouble(sdf, "robot_avoid_radius", 0.95);
    this->robotHardRadius_ = this->ReadDouble(sdf, "robot_hard_radius", 0.42);
    this->robotName_ = this->ReadString(sdf, "robot_name", "robot");
    this->progress_ = this->ReadDouble(sdf, "progress", 0.0);
    this->direction_ = this->ReadDouble(sdf, "direction", 1.0) < 0.0 ? -1.0 : 1.0;
    this->segment_ = static_cast<size_t>(std::max(0, this->ReadInt(sdf, "segment", 0)));

    if (sdf->HasElement("waypoint")) {
      auto waypoint = sdf->GetElement("waypoint");
      while (waypoint) {
        const auto x = waypoint->Get<double>("x");
        const auto y = waypoint->Get<double>("y");
        this->waypoints_.push_back({x, y});
        waypoint = waypoint->GetNextElement("waypoint");
      }
    }
    if (sdf->HasElement("static_obstacle")) {
      auto obstacle = sdf->GetElement("static_obstacle");
      while (obstacle) {
        this->staticObstacles_.push_back({
          obstacle->Get<double>("x"),
          obstacle->Get<double>("y"),
          obstacle->Get<double>("sx") * 0.5,
          obstacle->Get<double>("sy") * 0.5,
        });
        obstacle = obstacle->GetNextElement("static_obstacle");
      }
    }

    if (this->waypoints_.size() < 2 || this->speed_ <= 0.0) {
      this->enabled_ = false;
      return;
    }
    this->segment_ %= this->waypoints_.size();
    this->lastTime_ = this->world_->SimTime();
    this->connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&ScriptedActorPlugin::OnUpdate, this));
  }

private:
  void OnUpdate()
  {
    if (!this->enabled_) {
      return;
    }
    const auto now = this->world_->SimTime();
    const double dt = std::max(0.0, (now - this->lastTime_).Double());
    this->lastTime_ = now;
    this->simTime_ += dt;
    const size_t previousSegment = this->segment_;
    const double previousProgress = this->progress_;
    this->Advance(dt);
    auto pose = this->SteerAroundRobot(this->CurrentPose());
    if (this->TouchesRobot(pose) || this->TouchesStaticObstacle(pose)) {
      this->segment_ = previousSegment;
      this->progress_ = previousProgress;
      this->direction_ *= -1.0;
      this->laneOffset_ *= -1.0;
      this->Advance(dt * 0.8);
      pose = this->SteerAroundRobot(this->PushOutOfStaticObstacles(this->CurrentPose()));
      if (this->TouchesStaticObstacle(pose)) {
        this->segment_ = previousSegment;
        this->progress_ = previousProgress;
        pose = this->PushOutOfStaticObstacles(this->CurrentPose());
      }
    }
    this->model_->SetWorldPose(pose);
  }

  void Advance(double dt)
  {
    const auto & start = this->waypoints_[this->segment_];
    const auto & end = this->waypoints_[(this->segment_ + 1) % this->waypoints_.size()];
    const double dx = end.first - start.first;
    const double dy = end.second - start.second;
    const double distance = std::hypot(dx, dy);
    if (distance <= 1e-6) {
      this->segment_ = (this->segment_ + 1) % this->waypoints_.size();
      this->progress_ = 0.0;
      return;
    }
    const double speedModulation = 1.0 + 0.12 * std::sin(this->simTime_ * 0.75 + this->speedPhase_);
    this->progress_ += this->direction_ * this->speed_ * speedModulation * dt / distance;
    while (this->progress_ >= 1.0) {
      this->progress_ -= 1.0;
      this->segment_ = (this->segment_ + 1) % this->waypoints_.size();
    }
    while (this->progress_ < 0.0) {
      this->progress_ += 1.0;
      this->segment_ = (this->segment_ + this->waypoints_.size() - 1) % this->waypoints_.size();
    }
  }

  ignition::math::Pose3d CurrentPose() const
  {
    const auto & start = this->waypoints_[this->segment_];
    const auto & end = this->waypoints_[(this->segment_ + 1) % this->waypoints_.size()];
    const double xBase = start.first + (end.first - start.first) * this->progress_;
    const double yBase = start.second + (end.second - start.second) * this->progress_;
    const double yaw = std::atan2(end.second - start.second, end.first - start.first);
    const double lateral =
      this->laneOffset_ +
      this->swayAmplitude_ * std::sin(this->simTime_ * this->swayRate_ + this->swayPhase_) +
      0.025 * std::sin(this->simTime_ * (this->swayRate_ * 2.7) + this->swayPhase_ * 0.41);
    const double x = xBase - std::sin(yaw) * lateral;
    const double y = yBase + std::cos(yaw) * lateral;
    const double z = this->z_ + this->bobAmplitude_ *
      std::sin(this->simTime_ * this->swayRate_ * 2.0 + this->swayPhase_);
    return ignition::math::Pose3d(x, y, z, 0.0, 0.0, yaw);
  }

  ignition::math::Pose3d SteerAroundRobot(ignition::math::Pose3d pose) const
  {
    const auto robot = this->world_->ModelByName(this->robotName_);
    if (!robot) {
      return pose;
    }
    const auto robotPose = robot->WorldPose();
    double dx = pose.Pos().X() - robotPose.Pos().X();
    double dy = pose.Pos().Y() - robotPose.Pos().Y();
    double distance = std::hypot(dx, dy);
    if (distance >= this->robotAvoidRadius_) {
      return pose;
    }
    if (distance <= 1e-6) {
      const double side = this->laneOffset_ < 0.0 ? -1.0 : 1.0;
      dx = -std::sin(pose.Rot().Yaw()) * side;
      dy = std::cos(pose.Rot().Yaw()) * side;
      distance = 1.0;
    }
    const double push = (this->robotAvoidRadius_ - distance) * 1.15;
    pose.Pos().X(pose.Pos().X() + dx / distance * push);
    pose.Pos().Y(pose.Pos().Y() + dy / distance * push);
    return pose;
  }

  bool TouchesRobot(const ignition::math::Pose3d & pose) const
  {
    const auto robot = this->world_->ModelByName(this->robotName_);
    if (!robot) {
      return false;
    }
    const auto robotPose = robot->WorldPose();
    const double distance = std::hypot(
      pose.Pos().X() - robotPose.Pos().X(),
      pose.Pos().Y() - robotPose.Pos().Y());
    return distance < this->robotHardRadius_;
  }

  bool TouchesStaticObstacle(const ignition::math::Pose3d & pose) const
  {
    constexpr double padding = 0.36;
    for (const auto & obstacle : this->staticObstacles_) {
      if (
        std::abs(pose.Pos().X() - obstacle.x) <= obstacle.halfX + padding &&
        std::abs(pose.Pos().Y() - obstacle.y) <= obstacle.halfY + padding)
      {
        return true;
      }
    }
    return false;
  }

  ignition::math::Pose3d PushOutOfStaticObstacles(ignition::math::Pose3d pose) const
  {
    constexpr double padding = 0.42;
    for (const auto & obstacle : this->staticObstacles_) {
      const double dx = pose.Pos().X() - obstacle.x;
      const double dy = pose.Pos().Y() - obstacle.y;
      const double limitX = obstacle.halfX + padding;
      const double limitY = obstacle.halfY + padding;
      if (std::abs(dx) > limitX || std::abs(dy) > limitY) {
        continue;
      }
      const double pushX = limitX - std::abs(dx);
      const double pushY = limitY - std::abs(dy);
      if (pushX < pushY) {
        pose.Pos().X(obstacle.x + (dx < 0.0 ? -limitX : limitX));
      } else {
        pose.Pos().Y(obstacle.y + (dy < 0.0 ? -limitY : limitY));
      }
    }
    return pose;
  }

  static std::string ReadString(
    const sdf::ElementPtr & sdf, const std::string & name, const std::string & fallback)
  {
    if (!sdf->HasElement(name)) {
      return fallback;
    }
    return sdf->Get<std::string>(name);
  }

  static double ReadDouble(const sdf::ElementPtr & sdf, const std::string & name, double fallback)
  {
    return sdf->HasElement(name) ? sdf->Get<double>(name) : fallback;
  }

  static int ReadInt(const sdf::ElementPtr & sdf, const std::string & name, int fallback)
  {
    return sdf->HasElement(name) ? sdf->Get<int>(name) : fallback;
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr connection_;
  std::vector<std::pair<double, double>> waypoints_;
  struct StaticObstacle
  {
    double x;
    double y;
    double halfX;
    double halfY;
  };
  std::vector<StaticObstacle> staticObstacles_;
  gazebo::common::Time lastTime_;
  double speed_{0.8};
  double z_{0.8};
  double swayAmplitude_{0.06};
  double swayRate_{1.0};
  double swayPhase_{0.0};
  double bobAmplitude_{0.0};
  double speedPhase_{0.0};
  double avoidRadius_{0.55};
  double laneOffset_{0.0};
  double robotAvoidRadius_{0.95};
  double robotHardRadius_{0.42};
  double progress_{0.0};
  double direction_{1.0};
  double simTime_{0.0};
  size_t segment_{0};
  bool enabled_{true};
  std::string robotName_{"robot"};
};

GZ_REGISTER_MODEL_PLUGIN(ScriptedActorPlugin)
}  // namespace dynamic_nav_gazebo_plugins
