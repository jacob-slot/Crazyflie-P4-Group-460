#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace drone_plugin
{
  class DroneForcePlugin : public gz::sim::System, public rclcpp::Node
  {
  public:
    DroneForcePlugin()
      : Node("drone_force_plugin"), thrust_(0.0) {}

    void Configure(const gz::sim::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &,
                   gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
    {
      model_ = gz::sim::Model(entity);
      if (!model_.Valid())
      {
        RCLCPP_ERROR(this->get_logger(), "Model entity is invalid.");
        return;
      }

      // ROS 2 Subscription
      thrust_sub_ = this->create_subscription<std_msgs::msg::Float64>(
          "/drone/thrust", 10,
          std::bind(&DroneForcePlugin::OnThrustMsg, this, std::placeholders::_1));
    }

    void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &ecm) override
    {
      if (!model_.Valid())
        return;
      
      auto link = model_.LinkByName(ecm, "base_link");
      if (!link)
        return;
      
      // Apply force in the local +Z direction
      gz::math::Vector3d force(0, 0, thrust_);
      link->AddWorldForce(ecm, force);
    }

  private:
    void OnThrustMsg(const std_msgs::msg::Float64::SharedPtr msg)
    {
      thrust_ = msg->data;
    }

    gz::sim::Model model_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr thrust_sub_;
    double thrust_;
  };

  GZ_ADD_PLUGIN(DroneForcePlugin, gz::sim::System)
}
