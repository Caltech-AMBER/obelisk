#include "obelisk_std_msgs/msg/float_multi_array.hpp"
#include "obelisk_std_msgs/msg/u_int8_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <unsupported/Eigen/CXX11/Tensor>

namespace obelisk::utils::msgs {

    namespace internal {

        /**
         * @brief Convert a MultiArray message (data in row major layout) to an Eigen Tensor (column major).
         *
         * @param data the data to convert
         * @param dimensions the dimensions of the tensor
         * @param offset the offset to start reading the data
         * @return the Eigen Tensor
         */
        template <typename T, int Size>
        Eigen::Tensor<T, Size> MultiArrayToTensorImpl(const std::vector<T>& data,
                                                      const std::vector<std_msgs::msg::MultiArrayDimension>& dimensions,
                                                      size_t offset) {
            // Get the flat part of the data
            std::vector<T> flat_data(data.begin() + offset, data.end());

            if (dimensions.size() != Size) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("msg_conversion"),
                                    "Templated size does not match the size provided by the message!");
            }

            std::array<int, Size> sizes;
            for (int i = 0; i < Size; ++i) {
                sizes.at(i) = dimensions.at(i).size;
            }

            // Create the tensor dynamically
            Eigen::Tensor<T, Size> tensor;
            tensor.resize(sizes);

            // Populate the tensor with data
            std::array<int, Size> indices;
            for (int i = 0; i < tensor.size(); ++i) {
                int temp = i;
                for (int j = Size - 1; j >= 0; --j) {
                    indices[j] = temp % sizes[j];
                    temp /= sizes[j];
                }
                tensor(indices) = flat_data[i];
            }

            return tensor;
        }

        /**
         * @brief Convert an Eigen Tensor (column major) into a data vector (row major).
         *
         * @param tensor the tensor to convert
         * @return the vector
         */
        template <typename T, int Size> std::vector<T> TensorToMultiArrayData(const Eigen::Tensor<T, Size>& tensor) {
            std::vector<T> data(tensor.size());

            std::array<int, Size> dimSizes;
            for (int i = 0; i < Size; ++i) {
                dimSizes[i] = tensor.dimension(i);
            }

            for (int i = 0; i < tensor.size(); ++i) {
                std::array<int, Size> indices;
                int temp = i;
                for (int j = Size - 1; j >= 0; --j) {
                    indices[j] = temp % dimSizes[j];
                    temp /= dimSizes[j];
                }
                data[i] = tensor(indices);
            }

            return data;
        }

        /**
         * @brief Convert an Eigen Tensor (column major) into a layout vector.
         *
         * @param tensor the tensor to convert
         * @return the vector
         */
        template <typename T, int Size>
        std::vector<std_msgs::msg::MultiArrayDimension> TensorToMultiArrayLayout(const Eigen::Tensor<T, Size>& tensor) {
            std::vector<std_msgs::msg::MultiArrayDimension> layout;
            std_msgs::msg::MultiArrayDimension dim;
            dim.label  = "dim_" + std::to_string(Size);
            dim.size   = tensor.dimension(Size - 1);
            dim.stride = 1;

            layout.emplace_back(dim); // The stride for the last dimension
            for (int i = tensor.dimensions().size() - 2; i >= 0; --i) {
                dim.label  = "dim_" + std::to_string(i);
                dim.size   = tensor.dimension(i);
                dim.stride = layout.back().stride * tensor.dimension(i + 1);

                layout.emplace_back(dim);
            }
            std::reverse(layout.begin(), layout.end()); // Reverse to match dimension order

            return layout;
        }
    } // namespace internal

    // ****************************************** //
    // ********** MULTIARRAY TO TENSOR ********** //
    // ****************************************** //

    /**
     * @brief Convert MultiArray message to Eigen Tensor array.
     *
     * @param msg the message to convert
     * @return the Eigen Tensor
     */
    template <int Size>
    Eigen::Tensor<double, Size> MultiArrayToTensor(const obelisk_std_msgs::msg::FloatMultiArray& msg) {
        return internal::MultiArrayToTensorImpl<double, Size>(msg.data, msg.layout.dim, msg.layout.data_offset);
    }

    /**
     * @brief Convert MultiArray message to Eigen Tensor array.
     *
     * @param msg the message to convert
     * @return the Eigen Tensor
     */
    template <int Size>
    Eigen::Tensor<uint8_t, Size> MultiArrayToTensor(const obelisk_std_msgs::msg::UInt8MultiArray& msg) {
        return internal::MultiArrayToTensorImpl<uint8_t, Size>(msg.data, msg.layout.dim, msg.layout.data_offset);
    }

    // ****************************************** //
    // ********** TENSOR TO MULTIARRAY ********** //
    // ****************************************** //

    /**
     * @brief Convert an Eigen Tensor into a multiarray message.
     *
     * @param tensor the tensor to convert
     * @return the multiarray message
     */
    template <int Size>
    obelisk_std_msgs::msg::FloatMultiArray TensorToMultiArray(const Eigen::Tensor<double, Size>& tensor) {
        obelisk_std_msgs::msg::FloatMultiArray msg;
        msg.layout.data_offset = 0;

        msg.data       = internal::TensorToMultiArrayData<double, Size>(tensor);
        msg.layout.dim = internal::TensorToMultiArrayLayout<double, Size>(tensor);

        return msg;
    }

    /**
     * @brief Convert an Eigen Tensor into a multiarray message.
     *
     * @param tensor the tensor to convert
     * @return the multiarray message
     */
    template <int Size>
    obelisk_std_msgs::msg::UInt8MultiArray TensorToMultiArray(const Eigen::Tensor<uint8_t, Size>& tensor) {
        obelisk_std_msgs::msg::UInt8MultiArray msg;
        msg.layout.data_offset = 0;

        msg.data       = internal::TensorToMultiArrayData<uint8_t, Size>(tensor);
        msg.layout.dim = internal::TensorToMultiArrayLayout<uint8_t, Size>(tensor);

        return msg;
    }
} // namespace obelisk::utils::msgs
