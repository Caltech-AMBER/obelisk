#include <unsupported/Eigen/CXX11/Tensor>

#include "obelisk_std_msgs/msg/float_multi_array.hpp"
#include "obelisk_std_msgs/msg/u_int8_multi_array.hpp"

namespace obelisk::utils::msgs {
    namespace internal {
        /**
         * @brief Internal helper function to create tensors of arbitrary dimension.
         *
         * @param data the data to assign to the tensor
         * @param dims the dimensions of the tensor
         * @param Indices the index sequence to use for the parameter pack
         *
         * @return the Eigen Tensor
         */
        template <typename ScalarT, std::size_t N, std::size_t... Indices>
        Eigen::Tensor<ScalarT, N> CreateTensor(std::vector<ScalarT>& data, const std::array<int, N>& dims,
                                               std::index_sequence<Indices...>) {
            Eigen::TensorMap<Eigen::Tensor<ScalarT, N>> tensor(data.data(), dims[Indices]...);
            return tensor;
        }
    } // namespace internal

    /**
     * @brief Convert MultiArray message to Eigen Tensor array.
     *
     * @param msg the message to convert
     * @return the Eigen Tensor
     */
    template <int Size>
    Eigen::Tensor<double, Size> MutliArrayToTensor(const obelisk_std_msgs::msg::FloatMultiArray& msg) {

        // Get the flat part of the data
        std::vector<double> data(msg.data.begin() + msg.layout.data_offset, msg.data.end());

        if (msg.layout.dim.size() != Size) {
            // TODO: Consider just logging this, but without a node, we don't have access to a logger.
            throw std::runtime_error("Templated size does not match the size provided by the message!");
        }

        std::array<int, Size> sizes;
        for (int i = 0; i < Size; i++) {
            sizes.at(i) = msg.layout.dim.at(i).size;
        }

        auto tensor = internal::CreateTensor<double, Size>(data, sizes, std::make_index_sequence<Size>{});

        return tensor;
    }

    /**
     * @brief Convert MultiArray message to Eigen Tensor array.
     *
     * @param msg the message to convert
     * @return the Eigen Tensor
     */
    template <int Size>
    Eigen::Tensor<uint8_t, Size> MutliArrayToTensor(const obelisk_std_msgs::msg::UInt8MultiArray& msg) {

        // Get the flat part of the data
        std::vector<uint8_t> data(msg.data.begin() + msg.layout.data_offset, msg.data.end());

        if (msg.layout.dim.size() != Size) {
            // TODO: Consider just logging this, but without a node, we don't have access to a logger.
            throw std::runtime_error("Templated size does not match the size provided by the message!");
        }

        std::array<int, Size> sizes;
        for (int i = 0; i < Size; i++) {
            sizes.at(i) = msg.layout.dim.at(i).size;
        }

        auto tensor = internal::CreateTensor<uint8_t, Size>(data, sizes, std::make_index_sequence<Size>{});

        return tensor;
    }

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

        // Get data into flat vector
        msg.data.resize(tensor.size());
        std::copy(tensor.data(), tensor.data() + tensor.size(), msg.data.begin());

        // // Compute stride lengths
        std_msgs::msg::MultiArrayDimension dim;
        dim.label  = "dim_" + std::to_string(Size);
        dim.size   = tensor.dimension(Size - 1);
        dim.stride = 1;

        msg.layout.dim.emplace_back(dim); // The stride for the last dimension
        for (int i = tensor.dimensions().size() - 2; i >= 0; --i) {
            dim.label  = "dim_" + std::to_string(i);
            dim.size   = tensor.dimension(i);
            dim.stride = msg.layout.dim.back().stride * tensor.dimension(i + 1);

            msg.layout.dim.emplace_back(dim);
        }
        std::reverse(msg.layout.dim.begin(), msg.layout.dim.end()); // Reverse to match dimension order

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

        // Get data into flat vector
        msg.data.resize(tensor.size());
        std::copy(tensor.data(), tensor.data() + tensor.size(), msg.data.begin());

        // // Compute stride lengths
        std_msgs::msg::MultiArrayDimension dim;
        dim.label  = "dim_" + std::to_string(Size);
        dim.size   = tensor.dimension(Size - 1);
        dim.stride = 1;

        msg.layout.dim.emplace_back(dim); // The stride for the last dimension
        for (int i = tensor.dimensions().size() - 2; i >= 0; --i) {
            dim.label  = "dim_" + std::to_string(i);
            dim.size   = tensor.dimension(i);
            dim.stride = msg.layout.dim.back().stride * tensor.dimension(i + 1);

            msg.layout.dim.emplace_back(dim);
        }
        std::reverse(msg.layout.dim.begin(), msg.layout.dim.end()); // Reverse to match dimension order

        return msg;
    }
} // namespace obelisk::utils::msgs
