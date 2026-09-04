#pragma once
#include <cstdint>
#include <limits>

namespace obelisk {

    /**
     * Rejects spurious presses on an analog joystick axis used as a button (a trigger).
     *
     * Why this exists
     * ---------------
     * `UnitreeJoystick` reads its ESTOP and DAMPING bindings off the triggers with a bare
     * threshold and no hysteresis, and -- unlike the four mode transitions, which sit behind
     * a 0.5 s rate gate -- those two are evaluated un-gated. For a walking humanoid a
     * spurious ESTOP cuts the motors and drops the robot, so a false press is about as
     * costly as a missed one, which is what licenses filtering a safety input at all.
     *
     * Measured over 76 recorded hardware runs, the trigger-depth distribution is bimodal
     * with nothing in between: every one of the 55 deliberate pulls bottoms the trigger out
     * at exactly -1.000, while all 5 accidental contacts peak between -0.108 and -0.306.
     * The gap is 0.694 wide and empty. A press threshold placed mid-gap therefore separates
     * the two populations without rejecting a single real pull.
     *
     * Two mechanisms, both time-free (they count SAMPLES, never seconds, so the filter is
     * deterministic and does not depend on a clock or on the publish rate):
     *
     *   1. A Schmitt trigger. `press_at` is deep (default -0.5, mid-gap); `release_at` is
     *      shallow (default -0.15). The hysteresis band kills the chatter a finger resting
     *      on the trigger produces -- in one recorded run a single resting finger
     *      oscillating across -0.1 issued four separate ESTOP commands.
     *
     *   2. `confirm_samples` consecutive samples past `press_at` before the press is
     *      accepted (default 2). Depth alone rejects sustained shallow contact but not ONE
     *      corrupted reading: a single thrown sample at -0.51, or at -1.00, would fire
     *      ESTOP. Requiring two consecutive samples closes that and costs 1.1 ms at the
     *      median, because a trigger being physically pulled emits samples at ~930 Hz
     *      (event-driven `joy_node`) and the second one lands almost immediately. Counting
     *      samples rather than milliseconds is the whole point: a wall-clock sustain of
     *      even 10 ms costs ~100 ms in practice, since once the trigger STOPS moving
     *      `joy_node` goes quiet and the next sample is the 20 Hz autorepeat tick. Raise
     *      `autorepeat_rate` to bound that worst case.
     *
     * Deliberately NOT included: a refractory lockout after an accepted press. It would
     * suppress repeat pulls inside the window, which is harmless (ESTOP is a latched state,
     * not an event, so the first press already did the job) but also buys nothing -- and on
     * a safety path a mechanism that can swallow a second attempt needs a better reason
     * than tidiness.
     *
     * Deliberately NOT applied to digital buttons. `joy_node` is event-driven: a digital
     * press produces exactly ONE message and then nothing until release or the next
     * autorepeat tick, so `confirm_samples > 1` would reject short real presses outright.
     * The sample-density argument above holds only for an axis that physically sweeps.
     *
     * `notice_at` does not gate anything. It records the level the UNFILTERED code used
     * (-0.1), so the node can log precisely those presses that would have fired before and
     * no longer do -- which is both the operator's feedback that a brush was caught and the
     * evidence needed if the thresholds ever turn out to be too strict.
     */
    class AxisPressFilter {
      public:
        enum class Reject : uint8_t {
            NONE = 0,
            SHALLOW,      // never reached press_at -- a brush, or a finger resting on the trigger
            UNCONFIRMED,  // reached press_at but not for confirm_samples consecutive samples
        };

        AxisPressFilter() = default;

        /**
         * @param press_at        accept only below this axis value (more negative = deeper).
         * @param release_at      release above this value; must be >= press_at (shallower).
         * @param confirm_samples consecutive samples past press_at required to accept (>= 1).
         * @param notice_at       reporting-only level; a rejected excursion is only counted
         *                        (and logged) if it got at least this deep.
         */
        void Configure(float press_at, float release_at, int confirm_samples, float notice_at) {
            press_at_        = press_at;
            release_at_      = release_at;
            confirm_samples_ = confirm_samples > 1 ? confirm_samples : 1;
            notice_at_       = notice_at;
            Reset();
        }

        void Reset() {
            held_          = false;
            below_         = 0;
            in_excursion_  = false;
            accepted_this_ = false;
            peak_          = 0.0f;
            last_reject_   = Reject::NONE;
            last_peak_     = 0.0f;
            accepted_      = 0;
            rejected_      = 0;
        }

        /**
         * Feed ONE joystick sample. Returns the debounced pressed state.
         *
         * Call this exactly once per received message, for every binding, BEFORE any
         * branching on the result -- a filter that is only updated on the ticks where some
         * other binding happens to be idle carries a stale confirmation count.
         */
        bool Update(float value) {
            last_reject_       = Reject::NONE;
            const bool noticed = value < notice_at_;

            // An "excursion" is one contiguous stretch below notice_at_, i.e. one press
            // attempt as the unfiltered code would have seen it. Tracking it is what lets a
            // rejection be reported once, on the way out, with the depth actually reached.
            if (noticed) {
                if (!in_excursion_) {
                    in_excursion_  = true;
                    peak_          = value;
                    // If the axis is already held, this excursion belongs to a press that
                    // was accepted earlier and must not be reported as a rejection.
                    accepted_this_ = held_;
                } else if (value < peak_) {
                    peak_ = value;
                }
            }

            if (held_) {
                // >= not > : matches the reference Python implementation exactly, so the
                // offline tester's verdict on a recorded bag is the robot's verdict.
                if (value >= release_at_) {
                    held_  = false;
                    below_ = 0;
                }
            } else if (value <= press_at_) {
                if (++below_ >= confirm_samples_) {
                    held_          = true;
                    accepted_this_ = true;
                    ++accepted_;
                }
            } else {
                below_ = 0;
            }

            if (!noticed && in_excursion_) {
                in_excursion_ = false;
                if (!accepted_this_) {
                    ++rejected_;
                    last_reject_ = peak_ <= press_at_ ? Reject::UNCONFIRMED : Reject::SHALLOW;
                    last_peak_   = peak_;
                }
            }
            return held_;
        }

        bool held() const { return held_; }

        /// Non-NONE on exactly the one update that completed a rejected press attempt.
        Reject last_reject() const { return last_reject_; }

        /// Deepest axis value reached by the attempt that last_reject() describes.
        float last_peak() const { return last_peak_; }

        unsigned long accepted() const { return accepted_; }
        unsigned long rejected() const { return rejected_; }

        float press_at() const { return press_at_; }
        float release_at() const { return release_at_; }
        int confirm_samples() const { return confirm_samples_; }

      private:
        float press_at_        = -0.5f;
        float release_at_      = -0.15f;
        int confirm_samples_   = 2;
        float notice_at_       = -0.1f;

        bool held_             = false;
        int below_             = 0;
        bool in_excursion_     = false;
        bool accepted_this_    = false;
        float peak_            = 0.0f;

        Reject last_reject_    = Reject::NONE;
        float last_peak_       = 0.0f;
        unsigned long accepted_ = 0;
        unsigned long rejected_ = 0;
    };

}  // namespace obelisk
