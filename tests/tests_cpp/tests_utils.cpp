#include <catch2/catch_test_macros.hpp>
#include <iostream>

#include "msg_conversions.h"

TEST_CASE("Multiarray Testing", "[utils][msgs]") {
    // Generate 10 random tensors that are of dimension 5
    for (int l = 0; l < 10; l++) {
        Eigen::Tensor<double, 5> tensor2(4, 4, 5, 2, 3);
        for (int i = 0; i < tensor2.dimension(0); i++) {
            for (int j = 0; j < tensor2.dimension(1); j++) {
                for (int k = 0; k < tensor2.dimension(2); k++) {
                    for (int m = 0; m < tensor2.dimension(3); m++) {
                        for (int n = 0; n < tensor2.dimension(4); n++) {
                            tensor2(i, j, k, m, n) = rand() % 100;
                        }
                    }
                }
            }
        }

        auto msg                         = obelisk::utils::msgs::TensorToMultiArray<5>(tensor2);
        Eigen::Tensor<double, 5> tensor3 = obelisk::utils::msgs::MultiArrayToTensor<5>(msg);

        for (int i = 0; i < tensor2.dimension(0); i++) {
            for (int j = 0; j < tensor2.dimension(1); j++) {
                for (int k = 0; k < tensor2.dimension(2); k++) {
                    for (int m = 0; m < tensor2.dimension(3); m++) {
                        for (int n = 0; n < tensor2.dimension(4); n++) {
                            CHECK(tensor2(i, j, k, m, n) == tensor3(i, j, k, m, n));
                        }
                    }
                }
            }
        }
    }
}
