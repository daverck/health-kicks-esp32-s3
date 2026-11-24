#include <unity.h>
#include "activity_bout_tracker.h"
#include "../../src/services/activity_bout_tracker.cpp"

#ifndef ntohs
#define ntohs(x) ((uint16_t)((((uint16_t)(x) & 0x00FFU) << 8) | (((uint16_t)(x) & 0xFF00U) >> 8)))
#endif

void setUp(void) {}
void tearDown(void) {}

void test_single_bout_lifecycle_and_stillness_finalization(void) {
    ActivityBoutTracker tracker;
    tracker.begin();

    TEST_ASSERT_EQUAL_UINT32(0, tracker.getBoutCount());
    TEST_ASSERT_FALSE(tracker.isActiveBoutOpen());

    // Step 1 at T = 10s (10000ms), Walk, Cadence 80
    tracker.onStep(STATE_CODE_WALK, 10000, 80);
    TEST_ASSERT_TRUE(tracker.isActiveBoutOpen());
    TEST_ASSERT_EQUAL_UINT32(0, tracker.getBoutCount());

    // Step 2 at T = 15s (15000ms), Walk, Cadence 90
    tracker.onStep(STATE_CODE_WALK, 15000, 90);

    // Step 3 at T = 20s (20000ms), Walk, Cadence 100
    tracker.onStep(STATE_CODE_WALK, 20000, 100);

    // Tick at T = 25s (5s stillness < 10s timeout) -> bout should remain open
    tracker.onTick(25000);
    TEST_ASSERT_TRUE(tracker.isActiveBoutOpen());
    TEST_ASSERT_EQUAL_UINT32(0, tracker.getBoutCount());

    // Tick at T = 31s (11s stillness > 10s timeout) -> bout should finalize
    tracker.onTick(31000);
    TEST_ASSERT_FALSE(tracker.isActiveBoutOpen());
    TEST_ASSERT_EQUAL_UINT32(1, tracker.getBoutCount());

    std::vector<ActivityBout> bouts;
    tracker.getBouts(10, bouts);
    TEST_ASSERT_EQUAL_UINT32(1, bouts.size());

    // Check decoded fields (big-endian conversion)
    TEST_ASSERT_EQUAL_UINT16(10, ntohs(bouts[0].start_uptime_sec));
    TEST_ASSERT_EQUAL_UINT16(10, ntohs(bouts[0].duration_sec)); // 20s - 10s
    TEST_ASSERT_EQUAL_UINT16(3, ntohs(bouts[0].step_count));
    TEST_ASSERT_EQUAL_HEX8(0x01, bouts[0].activity_type); // Walk
    TEST_ASSERT_EQUAL_UINT8(90, bouts[0].avg_cadence);    // (80 + 90 + 100) / 3 = 90
}

void test_activity_transition_splits_bouts(void) {
    ActivityBoutTracker tracker;
    tracker.begin();

    // 3 steps walking from T = 10s to T = 12s
    tracker.onStep(STATE_CODE_WALK, 10000, 80);
    tracker.onStep(STATE_CODE_WALK, 11000, 82);
    tracker.onStep(STATE_CODE_WALK, 12000, 84);

    // Transition to Run at T = 13s, 2 steps running
    tracker.onStep(STATE_CODE_RUN, 13000, 150);
    tracker.onStep(STATE_CODE_RUN, 14000, 155);

    // Stillness at T = 25s (11s > 10s stillness)
    tracker.onTick(25000);

    TEST_ASSERT_EQUAL_UINT32(2, tracker.getBoutCount());

    std::vector<ActivityBout> bouts;
    tracker.getBouts(10, bouts);
    TEST_ASSERT_EQUAL_UINT32(2, bouts.size());

    // Bout 0: Walk
    TEST_ASSERT_EQUAL_HEX8(0x01, bouts[0].activity_type);
    TEST_ASSERT_EQUAL_UINT16(3, ntohs(bouts[0].step_count));

    // Bout 1: Run
    TEST_ASSERT_EQUAL_HEX8(0x02, bouts[1].activity_type);
    TEST_ASSERT_EQUAL_UINT16(2, ntohs(bouts[1].step_count));
    TEST_ASSERT_EQUAL_UINT16(13, ntohs(bouts[1].start_uptime_sec));
}

void test_ring_buffer_rollover_and_acknowledgment(void) {
    ActivityBoutTracker tracker;
    tracker.begin();

    // Simulate 130 discrete bouts (each 2 steps, closed by flush)
    for (uint32_t i = 0; i < 130; ++i) {
        uint32_t t = i * 20000;
        tracker.onStep(STATE_CODE_WALK, t, 100);
        tracker.onStep(STATE_CODE_WALK, t + 1000, 100);
        tracker.flushActiveBout(t + 2000);
    }

    // Buffer capacity is 128
    TEST_ASSERT_EQUAL_UINT32(128, tracker.getBoutCount());

    // Acknowledge 30 bouts
    tracker.acknowledgeCount(30);
    TEST_ASSERT_EQUAL_UINT32(98, tracker.getBoutCount());

    // Acknowledge remaining 100 (more than 98) -> should clear to 0
    tracker.acknowledgeCount(100);
    TEST_ASSERT_EQUAL_UINT32(0, tracker.getBoutCount());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_single_bout_lifecycle_and_stillness_finalization);
    RUN_TEST(test_activity_transition_splits_bouts);
    RUN_TEST(test_ring_buffer_rollover_and_acknowledgment);
    return UNITY_END();
}
