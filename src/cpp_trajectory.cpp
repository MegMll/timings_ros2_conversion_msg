#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "ros_interface_mpc/msg/initial_state.hpp"
#include "ros_interface_mpc/msg/trajectory.hpp"
#include "go2_benchmark_msg/utils.h"

using std::placeholders::_1;

class TrajectoryPublisher : public rclcpp::Node
{
  public:
    TrajectoryPublisher()
    : Node("cpp_trajectory")
    {
      publisher_ = this->create_publisher<ros_interface_mpc::msg::Trajectory>("/trajectory", 10);
      subscription_ = this->create_subscription<ros_interface_mpc::msg::InitialState>(
      "/initial_state", 10, std::bind(&TrajectoryPublisher::state_callback, this, _1));

      // this->declare_parameter("time_length", 10);
      int time_length = 10;
      int nq = 10;
      
      x0_ = Eigen::VectorXd::Zero(time_length);  
      k0_ = Eigen::MatrixXd::Zero(time_length, nq);
      us_ = Eigen::MatrixXd::Zero(time_length, nq);
      xs_ = Eigen::MatrixXd::Zero(time_length, nq);
      ddq = Eigen::MatrixXd::Zero(time_length, nq);
      forces = Eigen::MatrixXd::Zero(time_length, nq);
      contact_states = Eigen::MatrixXi::Zero(time_length, nq);
      RCLCPP_INFO(this->get_logger(), "cpp trajectory instantiated");

    }

  private:
    void state_callback(const ros_interface_mpc::msg::InitialState::SharedPtr msg)
    {
      x0_ = Eigen::Map<const Eigen::VectorXd>(msg->x0.data(), msg->x0.size());

      k0_.setRandom();
      us_.setRandom();
      xs_.setRandom();
      ddq.setRandom();
      forces.setRandom();
      contact_states.setRandom();

      ros_interface_mpc::msg::Trajectory msg2send;
      // Fill the x0 field of the message
      transform_eigen_into_multiarray(k0_, msg2send.k0);
      transform_eigen_into_multiarray(us_, msg2send.us);
      transform_eigen_into_multiarray(xs_, msg2send.xs);
      transform_eigen_into_multiarray(ddq, msg2send.ddqs);
      transform_eigen_into_multiarray(forces, msg2send.forces);
      transform_eigen_into_multiarray(contact_states, msg2send.contact_states);
      
      state_time.sec = static_cast<int32_t>(clock.now().seconds());
      state_time.nanosec = static_cast<uint32_t>(clock.now().nanoseconds());
      msg2send.stamp = state_time;

      publisher_->publish(msg2send);

    }

    rclcpp::Publisher<ros_interface_mpc::msg::Trajectory>::SharedPtr publisher_;
    rclcpp::Subscription<ros_interface_mpc::msg::InitialState>::SharedPtr subscription_;   

    Eigen::VectorXd x0_;
    Eigen::MatrixXd k0_, us_, xs_, ddq, forces;
    Eigen::MatrixXi contact_states;

    rclcpp::Clock clock;
    builtin_interfaces::msg::Time state_time;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}