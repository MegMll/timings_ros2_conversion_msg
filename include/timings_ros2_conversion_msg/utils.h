#include <Eigen/Dense>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int8_multi_array.hpp" 
#include "std_msgs/msg/multi_array_dimension.hpp"

void transform_multiarray_into_eigen(const std_msgs::msg::Float64MultiArray& multi_array_msg, Eigen::MatrixXd& out_mat);

void transform_multiarray_into_eigen(const std_msgs::msg::Int8MultiArray& multi_array_msg, Eigen::MatrixXi& out_mat);

void transform_eigen_into_multiarray(Eigen::MatrixXd& mat, std_msgs::msg::Float64MultiArray& multi_array_msg);

void transform_eigen_into_multiarray(Eigen::MatrixXi& mat, std_msgs::msg::Int8MultiArray& multi_array_msg);
