#include <catch2/catch_test_macros.hpp>

#include <vector>

#include "joy_debounce.h"

using obelisk::AxisPressFilter;

namespace {

    // Replays a trace and returns how many times the filter latched a NEW press.
    //
    // last_reject() is a ONE-SHOT -- set on the single update that completes a rejected
    // attempt and cleared by the next one, exactly as getFilteredButton() consumes it --
    // so it is captured here per sample rather than read after the loop.
    int CountAccepts(AxisPressFilter& f, const std::vector<float>& trace,
                     AxisPressFilter::Reject* why = nullptr) {
        int accepts = 0;
        bool prev   = false;
        for (float v : trace) {
            const bool held = f.Update(v);
            if (held && !prev) ++accepts;
            if (why != nullptr && f.last_reject() != AxisPressFilter::Reject::NONE) {
                *why = f.last_reject();
            }
            prev = held;
        }
        return accepts;
    }

    AxisPressFilter MakeFilter() {
        AxisPressFilter f;
        f.Configure(-0.5f, -0.15f, 2, -0.1f);  // the shipped defaults
        return f;
    }

}  // namespace

// The two populations this filter exists to separate. Every trace below is either taken
// from, or modelled directly on, the 76 hardware runs analysed in ctrl_logs/ -- the
// rejected ones are real recorded events, not invented ones.

TEST_CASE("A deliberate pull is accepted on the second deep sample", "[joy_debounce]") {
    auto f = MakeFilter();
    // A trigger physically being pulled sweeps through its travel emitting samples at
    // ~930 Hz, so the confirming sample lands ~1 ms after the first.
    REQUIRE(CountAccepts(f, {1.0f, 0.6f, 0.1f, -0.55f, -0.82f, -1.0f, -1.0f, -0.3f, 1.0f}) == 1);
    REQUIRE(f.rejected() == 0);
}

TEST_CASE("A brush that never reaches the press threshold is rejected", "[joy_debounce]") {
    auto f = MakeFilter();
    // bec1_2026-09-03_17-58-58 @398.772: 45 ms at 5% of travel, read as ESTOP by the
    // unfiltered code; the robot was walking at 0.85 m/s and fell.
    auto why = AxisPressFilter::Reject::NONE;
    REQUIRE(CountAccepts(f, {1.0f, -0.108f, -0.09f, 1.0f}, &why) == 0);
    REQUIRE(f.rejected() == 1);
    REQUIRE(why == AxisPressFilter::Reject::SHALLOW);
}

TEST_CASE("Chatter from a finger resting on the trigger is rejected", "[joy_debounce]") {
    auto f = MakeFilter();
    // nav_2026-08-17_08-46-17 @134.2-134.7: one resting finger oscillating across the
    // old -0.1 threshold issued four separate ESTOP commands.
    REQUIRE(CountAccepts(f, {1.0f, -0.12f, -0.05f, -0.14f, -0.03f, -0.11f, 1.0f}) == 0);
    REQUIRE(f.rejected() > 0);
}

TEST_CASE("A single thrown sample past the threshold is rejected", "[joy_debounce]") {
    // Depth alone cannot survive one corrupted reading; confirm_samples is what does.
    SECTION("just past the threshold") {
        auto f   = MakeFilter();
        auto why = AxisPressFilter::Reject::NONE;
        REQUIRE(CountAccepts(f, {1.0f, 1.0f, -0.51f, 1.0f, 1.0f}, &why) == 0);
        REQUIRE(why == AxisPressFilter::Reject::UNCONFIRMED);
    }
    SECTION("at full travel") {
        auto f   = MakeFilter();
        auto why = AxisPressFilter::Reject::NONE;
        REQUIRE(CountAccepts(f, {1.0f, 1.0f, -1.0f, 1.0f, 1.0f}, &why) == 0);
        REQUIRE(why == AxisPressFilter::Reject::UNCONFIRMED);
    }
    SECTION("two consecutive corrupted samples still pass -- the documented limit") {
        auto f = MakeFilter();
        REQUIRE(CountAccepts(f, {1.0f, 1.0f, -1.0f, -1.0f, 1.0f}) == 1);
    }
}

TEST_CASE("Hysteresis holds a press through a partial release", "[joy_debounce]") {
    auto f = MakeFilter();
    // Inside the band the press must neither drop nor re-trigger.
    REQUIRE(CountAccepts(f, {-1.0f, -1.0f, -0.2f, -0.9f, -0.9f, 1.0f}) == 1);
    REQUIRE(f.accepted() == 1);
}

TEST_CASE("A genuine second pull after a full release is accepted", "[joy_debounce]") {
    auto f = MakeFilter();
    // No refractory lockout on this path: a repeat attempt must never be swallowed.
    REQUIRE(CountAccepts(f, {-1.0f, -1.0f, 1.0f, 1.0f, -1.0f, -1.0f, 1.0f}) == 2);
}

TEST_CASE("confirm_samples = 1 restores the unfiltered single-sample behaviour",
          "[joy_debounce]") {
    AxisPressFilter f;
    f.Configure(-0.1f, -0.1f, 1, -0.1f);
    REQUIRE(CountAccepts(f, {1.0f, -0.108f, 1.0f}) == 1);
}

TEST_CASE("Configure clamps confirm_samples to at least one", "[joy_debounce]") {
    AxisPressFilter f;
    f.Configure(-0.5f, -0.15f, 0, -0.1f);
    REQUIRE(f.confirm_samples() == 1);
    REQUIRE(CountAccepts(f, {1.0f, -1.0f, 1.0f}) == 1);
}
