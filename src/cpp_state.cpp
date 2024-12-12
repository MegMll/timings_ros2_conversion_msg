#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int8_multi_array.hpp" 
#include "std_msgs/msg/multi_array_dimension.hpp"

#include "ros_interface_mpc/msg/initial_state.hpp"
#include "ros_interface_mpc/msg/trajectory.hpp"

#include <Eigen/Dense>
#include <vector>

#include "go2_benchmark_msg/utils.h"

using std::placeholders::_1;


class StatePublisher : public rclcpp::Node
{
  public:
    StatePublisher()
    : Node("cpp_state")
    {
      publisher_ = this->create_publisher<ros_interface_mpc::msg::InitialState>("/initial_state", 10);
      subscription_ = this->create_subscription<ros_interface_mpc::msg::Trajectory>(
      "/trajectory", 10, std::bind(&StatePublisher::trajectory_callback, this, _1));

      this->declare_parameter("nq", 10);
      this->declare_parameter("time_length", 10);
      int time_length = this->get_parameter("time_length").as_int();
      int nq = this->get_parameter("nq").as_int();
      x0_ = Eigen::VectorXd::Zero(time_length);  
      k0_ = Eigen::MatrixXd::Zero(time_length, nq);
      us_ = Eigen::MatrixXd::Zero(time_length, nq);
      xs_ = Eigen::MatrixXd::Zero(time_length, nq);
      ddq = Eigen::MatrixXd::Zero(time_length, nq);
      forces = Eigen::MatrixXd::Zero(time_length, nq);
      contact_states = Eigen::MatrixXi::Zero(time_length, nq);

      RCLCPP_INFO(this->get_logger(), "cpp state instantiated");

      x0_.setRandom();
      ros_interface_mpc::msg::InitialState msg2send;
      // Convert Eigen::VectorXd to std::vector<double> (or any suitable type depending on the message)
      std::vector<double> x0_vector(x0_.data(), x0_.data() + x0_.size());

      // Fill the x0 field of the message
      msg2send.x0 = x0_vector;
      msg2send.stamp = clock.now();
      publisher_->publish(msg2send);
    }

  private:
    void trajectory_callback(const ros_interface_mpc::msg::Trajectory::SharedPtr msg)
    {
      // Process trajectory message
      transform_multiarray_into_eigen(msg->us, us_);
      transform_multiarray_into_eigen(msg->xs, xs_);
      transform_multiarray_into_eigen(msg->k0, k0_);
      transform_multiarray_into_eigen(msg->ddqs, ddq);
      transform_multiarray_into_eigen(msg->forces, forces);
      transform_multiarray_into_eigen(msg->contact_states, contact_states);
      trajectory_stamp_ = msg->stamp;

      x0_.setRandom();
      ros_interface_mpc::msg::InitialState msg2send;
      std::vector<double> x0_vector(x0_.data(), x0_.data() + x0_.size());

      // Fill the x0 field of the message
      msg2send.x0 = x0_vector;
      state_time.sec = static_cast<int32_t>(clock.now().seconds());
      state_time.nanosec = static_cast<uint32_t>(clock.now().nanoseconds());
      msg2send.stamp = state_time;
      publisher_->publish(msg2send);

      // Compute delay
      double delay = static_cast<double>(static_cast<int32_t>(clock.now().seconds()) - msg->stamp.sec) +
                     static_cast<double>(static_cast<int32_t>(clock.now().nanoseconds()) - msg->stamp.nanosec) / 1e9;

      total_delay_ += delay;
      delay_count_++;
      double average_delay = total_delay_ / delay_count_;

      RCLCPP_INFO(this->get_logger(), "Average delay: %.6f seconds", average_delay);
    }
    
    rclcpp::Publisher<ros_interface_mpc::msg::InitialState>::SharedPtr publisher_;
    rclcpp::Subscription<ros_interface_mpc::msg::Trajectory>::SharedPtr subscription_;
    
    builtin_interfaces::msg::Time trajectory_stamp_;
    builtin_interfaces::msg::Time state_time;

    Eigen::VectorXd x0_;
    Eigen::MatrixXd k0_, us_, xs_, ddq, forces;
    Eigen::MatrixXi contact_states;

    rclcpp::Clock clock;
    double total_delay_;
    int delay_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatePublisher>());
  rclcpp::shutdown();
  return 0;
}