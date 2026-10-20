#include <unity.h>
#include "inactivity_monitor.h"
#include "../../src/services/inactivity_monitor.cpp"

uint32_t mockMillisValue = 1000;
MockSerial Serial;

void setUp(void) {
    mockMillisValue = 1000;
}

void tearDown(void) {}

void configureMonitor(InactivityMonitor& monitor, uint16_t thresholdSec = 10) {
    monitor.begin();
    monitor.configure(true, thresholdSec, 0);
}

void test_idle_steps_do_not_reset_inactivity_timer(void) {
    InactivityMonitor monitor;
    configureMonitor(monitor);

    monitor.processStepActivity(2, STATE_CODE_IDLE, 8000);
    TEST_ASSERT_EQUAL_UINT32(1000, monitor.getLastStepTimestampMs());
    TEST_ASSERT_FALSE(monitor.isAlertTriggered());

    monitor.processStepActivity(2, STATE_CODE_IDLE, 11000);
    TEST_ASSERT_TRUE(monitor.isAlertTriggered());
}

void test_sitting_steps_do_not_reset_inactivity_timer(void) {
    InactivityMonitor monitor;
    configureMonitor(monitor);

    monitor.processStepActivity(2, STATE_CODE_SIT_DOWN, 8000);
    TEST_ASSERT_EQUAL_UINT32(1000, monitor.getLastStepTimestampMs());
    TEST_ASSERT_FALSE(monitor.isAlertTriggered());

    monitor.processStepActivity(2, STATE_CODE_SIT_DOWN, 11000);
    TEST_ASSERT_TRUE(monitor.isAlertTriggered());
}

void test_all_locomotion_steps_reset_inactivity_timer(void) {
    const uint8_t activeStates[] = {
        STATE_CODE_WALK,
        STATE_CODE_RUN,
        STATE_CODE_STAIRS_UP,
        STATE_CODE_STAIRS_DOWN,
    };

    for (uint8_t activityState : activeStates) {
        InactivityMonitor monitor;
        configureMonitor(monitor);

        monitor.processStepActivity(1, activityState, 5000);
        TEST_ASSERT_EQUAL_UINT32(5000, monitor.getLastStepTimestampMs());
    }
}

void test_walking_resumes_alert_after_ten_active_steps(void) {
    InactivityMonitor monitor;
    configureMonitor(monitor, 1);
    monitor.processStepActivity(0, STATE_CODE_IDLE, 2000);
    TEST_ASSERT_TRUE(monitor.isAlertTriggered());

    monitor.processStepActivity(20, STATE_CODE_SIT_DOWN, 2100);
    TEST_ASSERT_TRUE(monitor.isAlertTriggered());
    TEST_ASSERT_EQUAL_UINT32(1000, monitor.getLastStepTimestampMs());

    monitor.processStepActivity(29, STATE_CODE_WALK, 2200);
    TEST_ASSERT_TRUE(monitor.isAlertTriggered());
    TEST_ASSERT_EQUAL_UINT32(2200, monitor.getLastStepTimestampMs());

    monitor.processStepActivity(30, STATE_CODE_WALK, 2300);
    TEST_ASSERT_FALSE(monitor.isAlertTriggered());
    TEST_ASSERT_EQUAL_UINT32(2300, monitor.getLastStepTimestampMs());
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_idle_steps_do_not_reset_inactivity_timer);
    RUN_TEST(test_sitting_steps_do_not_reset_inactivity_timer);
    RUN_TEST(test_all_locomotion_steps_reset_inactivity_timer);
    RUN_TEST(test_walking_resumes_alert_after_ten_active_steps);
    return UNITY_END();
}
