
#include "go2_benchmark_msg/utils.h"

void transform_multiarray_into_eigen(const std_msgs::msg::Float64MultiArray& multi_array_msg, Eigen::MatrixXd& out_mat)
{
    const float h = multi_array_msg.layout.dim[0].size;
    const float w = multi_array_msg.layout.dim[1].size;

    // Below are a few basic Eigen demos:
    const std::vector<double>& data = multi_array_msg.data;
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mat(data.data(), h, w);

    out_mat = mat;
}

void transform_multiarray_into_eigen(const std_msgs::msg::Int8MultiArray& multi_array_msg, Eigen::MatrixXi& out_mat)
{
    const float h = multi_array_msg.layout.dim[0].size;
    const float w = multi_array_msg.layout.dim[1].size;

    // Below are a few basic Eigen demos:
    const std::vector<signed char>& data = multi_array_msg.data;
    Eigen::Map<const Eigen::Matrix<signed char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mat(data.data(), h, w);
    
    out_mat = mat.cast<int>();
}

void transform_eigen_into_multiarray(Eigen::MatrixXd& mat, std_msgs::msg::Float64MultiArray& multi_array_msg)
{
    // Set the dimensions of the MultiArray
    multi_array_msg.layout.dim.resize(2);
    multi_array_msg.layout.dim[0].label = "rows";
    multi_array_msg.layout.dim[0].size = mat.rows();
    multi_array_msg.layout.dim[0].stride = mat.rows() * mat.cols();
    multi_array_msg.layout.dim[1].label = "cols";
    multi_array_msg.layout.dim[1].size = mat.cols();
    multi_array_msg.layout.dim[1].stride = mat.cols();

    std::vector<double> vec;
    vec.resize(mat.size());
    Eigen::Map<Eigen::VectorXd> mvec(mat.data(), mat.size());
    Eigen::VectorXd::Map(&vec[0], mvec.size()) = mvec;
    multi_array_msg.data = vec;
}

void transform_eigen_into_multiarray(Eigen::MatrixXi& mat, std_msgs::msg::Int8MultiArray& multi_array_msg)
{
    // Set the dimensions of the MultiArray
    multi_array_msg.layout.dim.resize(2);
    multi_array_msg.layout.dim[0].label = "rows";
    multi_array_msg.layout.dim[0].size = mat.rows();
    multi_array_msg.layout.dim[0].stride = mat.rows() * mat.cols();
    multi_array_msg.layout.dim[1].label = "cols";
    multi_array_msg.layout.dim[1].size = mat.cols();
    multi_array_msg.layout.dim[1].stride = mat.cols();

    std::vector<signed char> vec;
    vec.resize(mat.size());
    Eigen::Map<Eigen::VectorXi> mvec(mat.data(), mat.size());
    // Map the vector with the same size and copy values from Eigen matrix
    for (int i = 0; i < mvec.size(); ++i) {
        vec[i] = static_cast<signed char>(mvec.data()[i]);
    }
    multi_array_msg.data = (vec);
}