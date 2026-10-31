#include <unity.h>
#include <stdint.h>
#include <string.h>
#include "invariants.h"
#include "activity_model_generated.h"

void setUp(void) {
    // Set up before each test
}

void tearDown(void) {
    // Clean up after each test
}

void test_imu_raw_frame_binary_packing(void) {
    // Invariant: Compressed raw IMU frame must be strictly packed to 14 bytes
    TEST_ASSERT_EQUAL_UINT32(14, sizeof(ImuRawFrame));
}

void test_burst_packet_header_binary_packing(void) {
    // Invariant: Standard burst packet header must be strictly packed to 4 bytes
    TEST_ASSERT_EQUAL_UINT32(4, sizeof(BurstPacketHeader));
}

void test_step_counter_payload_binary_packing(void) {
    // Invariant: Pedometer payload on Characteristic 0006 must be strictly packed to 15 bytes
    TEST_ASSERT_EQUAL_UINT32(15, sizeof(StepCounterPayload));
}

void test_inactivity_config_payload_binary_packing(void) {
    // Invariant: Inactivity payload on Characteristic 0003 must be strictly packed to 6 bytes
    TEST_ASSERT_EQUAL_UINT32(6, sizeof(InactivityConfigPayload));
}

void test_activity_state_codes_contract(void) {
    // Invariant: GATT activity state codes contract compliance
    TEST_ASSERT_EQUAL_HEX8(0x00, STATE_CODE_IDLE);
    TEST_ASSERT_EQUAL_HEX8(0x01, STATE_CODE_WALK);
    TEST_ASSERT_EQUAL_HEX8(0x02, STATE_CODE_RUN);
    TEST_ASSERT_EQUAL_HEX8(0x03, STATE_CODE_STAIRS_UP);
    TEST_ASSERT_EQUAL_HEX8(0x04, STATE_CODE_STAIRS_DOWN);
    TEST_ASSERT_EQUAL_HEX8(0x05, STATE_CODE_SIT_DOWN);
    TEST_ASSERT_EQUAL_HEX8(0x06, STATE_CODE_STAND_UP);
    TEST_ASSERT_EQUAL_HEX8(0x07, STATE_CODE_LYING_SUPINE);
    TEST_ASSERT_EQUAL_HEX8(0x08, STATE_CODE_LYING_PRONE);
    TEST_ASSERT_EQUAL_HEX8(0x09, STATE_CODE_LYING_LATERAL);
    TEST_ASSERT_EQUAL_HEX8(0x0A, STATE_CODE_JUMP);
    TEST_ASSERT_EQUAL_HEX8(0x0B, STATE_CODE_LIE_DOWN);
    TEST_ASSERT_EQUAL_HEX8(0x10, STATE_CODE_FALL_FORWARD);
    TEST_ASSERT_EQUAL_HEX8(0x11, STATE_CODE_FALL_BACKWARD);
    TEST_ASSERT_EQUAL_HEX8(0x12, STATE_CODE_FALL_LATERAL);
    TEST_ASSERT_EQUAL_HEX8(0x13, STATE_CODE_FALL_RECOVERY);
    TEST_ASSERT_EQUAL_HEX8(0x20, STATE_CODE_INACTIVITY_ALERT);
}

void test_activity_model_dimensions(void) {
    // Invariant: TinyML feature dimension and class count
    TEST_ASSERT_EQUAL_INT(24, MODEL_FEATURE_COUNT);
    TEST_ASSERT_EQUAL_INT(16, MODEL_CLASS_COUNT);
    TEST_ASSERT_EQUAL_INT(57, MODEL_WINDOW_SAMPLES);
}

void test_activity_model_fall_classes_mask(void) {
    // Invariant: Fall classes identified correctly in model mask
    // Index 0: fall_backward (true)
    // Index 1: fall_forward (true)
    // Index 2: fall_lateral (true)
    // Index 3: fall_recovery (false)
    // Index 4: idle (false)
    // Index 5: jump (false)
    // Index 6: lie_down (false)
    // Index 7: lying_lateral (false)
    // Index 8: lying_prone (false)
    // Index 9: lying_supine (false)
    // Index 10: run (false)
    // Index 11: sit_down (false)
    // Index 12: stairs_down (false)
    // Index 13: stairs_up (false)
    // Index 14: stand_up (false)
    // Index 15: walk (false)
    TEST_ASSERT_TRUE(MODEL_IS_FALL_CLASS[0]);
    TEST_ASSERT_TRUE(MODEL_IS_FALL_CLASS[1]);
    TEST_ASSERT_TRUE(MODEL_IS_FALL_CLASS[2]);
    TEST_ASSERT_FALSE(MODEL_IS_FALL_CLASS[3]);
    TEST_ASSERT_FALSE(MODEL_IS_FALL_CLASS[4]);
    TEST_ASSERT_FALSE(MODEL_IS_FALL_CLASS[5]);  // jump
    TEST_ASSERT_FALSE(MODEL_IS_FALL_CLASS[6]);  // lie_down
    TEST_ASSERT_FALSE(MODEL_IS_FALL_CLASS[7]);  // lying_lateral
    TEST_ASSERT_FALSE(MODEL_IS_FALL_CLASS[8]);  // lying_prone
    TEST_ASSERT_FALSE(MODEL_IS_FALL_CLASS[9]);  // lying_supine
    TEST_ASSERT_FALSE(MODEL_IS_FALL_CLASS[10]); // run
    TEST_ASSERT_FALSE(MODEL_IS_FALL_CLASS[11]); // sit_down
    TEST_ASSERT_FALSE(MODEL_IS_FALL_CLASS[12]); // stairs_down
    TEST_ASSERT_FALSE(MODEL_IS_FALL_CLASS[13]); // stairs_up
    TEST_ASSERT_FALSE(MODEL_IS_FALL_CLASS[14]); // stand_up
    TEST_ASSERT_FALSE(MODEL_IS_FALL_CLASS[15]); // walk
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_imu_raw_frame_binary_packing);
    RUN_TEST(test_burst_packet_header_binary_packing);
    RUN_TEST(test_step_counter_payload_binary_packing);
    RUN_TEST(test_inactivity_config_payload_binary_packing);
    RUN_TEST(test_activity_state_codes_contract);
    RUN_TEST(test_activity_model_dimensions);
    RUN_TEST(test_activity_model_fall_classes_mask);
    return UNITY_END();
}
