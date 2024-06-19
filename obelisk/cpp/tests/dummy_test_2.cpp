/*
 * Obelisk Software Framework.
 * Developed in the Amber Lab at Caltech.
 * Copyright (c) 2024 Amber Lab. All rights reserved.
 */

#include <catch2/catch_test_macros.hpp>

#include "place_holder.h"

TEST_CASE("Dummy test 2", "[dummy-test]") {
    int j = 3;
    REQUIRE(j == 3);

    SECTION("Test section") {
        REQUIRE(obelisk::lib1::Func1(2,2) == 4);
    }
}